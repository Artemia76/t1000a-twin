#!/usr/bin/env python3
"""Lance un ou plusieurs jumeaux T1000-A en temps réel et les expose aux vrais outils par TCP.

Pour chaque nœud i (0, 1, …) :
  * port PORT+i      : protocole compagnon MeshCore en TCP (trames '<' len16 données / '>' len16 données),
                       comme un nœud Wi-Fi : meshcore-cli -t 127.0.0.1 -p 5000
                       La connexion TCP = le téléphone virtuel se connecte, s'appaire (code) et s'abonne.
  * port PORT+100+i  : console BLE de secours (BleConsole/GUS) en texte : nc 127.0.0.1 5100
Les nœuds partagent le même éther LoRa : ils s'entendent entre eux.

Usage :
  run_twin.py firmware.elf [-n 2] [--port 5000] [--flash-dir DIR] [--speed 1.0] [--gps 48.85,2.29]
              [--battery 3900] [--trace]
--flash-dir : la Flash de chaque nœud est chargée depuis DIR/<nom>.bin et réécrite à l'arrêt (persistance).
--speed     : 1.0 = temps réel, 0 = aussi vite que possible.
"""
import argparse, os, selectors, signal, socket, struct, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from twin import T1000A, World                                  # noqa: E402
from twin.companion import Companion, NUS_RX, NUS_TX            # noqa: E402

_B = [0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80, 0x0A, 0x46, 0x44, 0xD3]
GUS_TX, GUS_RX = bytes(_B + [0x02, 0x02, 0xED, 0xA6]), bytes(_B + [0x03, 0x02, 0xED, 0xA6])


class Link:
    """Une connexion TCP rattachée au téléphone virtuel d'un nœud."""

    def __init__(self, bridge, node, sock, kind):
        self.bridge, self.node, self.sock, self.kind = bridge, node, sock, kind
        self.buf = bytearray()
        self.ready = False
        sock.setblocking(False)

    def start(self):
        ph = self.node.phone
        if not ph.connected:
            c = Companion(self.node)
            c.start()                          # connexion + appairage + abonnement NUS (avance le temps)
        if self.kind == 'console':
            ph.subscribe(GUS_TX)
        self.ready = True

    def on_notify(self, uuid, data):
        if not self.ready:
            return
        try:
            if self.kind == 'companion' and uuid == NUS_TX:
                self.sock.sendall(b'>' + struct.pack('<H', len(data)) + data)
            elif self.kind == 'console' and uuid == GUS_TX:
                self.sock.sendall(data)
        except OSError:
            pass

    def feed(self, data):
        self.buf += data
        ph = self.node.phone
        if self.kind == 'console':
            while b'\n' in self.buf:
                line, _, rest = bytes(self.buf).partition(b'\n')
                self.buf = bytearray(rest)
                ph.write(GUS_RX, line.rstrip(b'\r') + b'\r\n')
            return
        while len(self.buf) >= 3:
            if self.buf[0] != 0x3C:                    # '<'
                del self.buf[0]
                continue
            n = struct.unpack_from('<H', self.buf, 1)[0]
            if len(self.buf) < 3 + n:
                break
            frame = bytes(self.buf[3:3 + n])
            del self.buf[:3 + n]
            ph.write(NUS_RX, frame)


class Bridge:
    def __init__(self, world, port):
        self.world = world
        self.sel = selectors.DefaultSelector()
        self.links = []
        for i, node in enumerate(world.nodes):
            for kind, p in (('companion', port + i), ('console', port + 100 + i)):
                s = socket.socket()
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.bind(('127.0.0.1', p))
                s.listen(1)
                s.setblocking(False)
                self.sel.register(s, selectors.EVENT_READ, ('listen', node, kind))
                print(f'  {node.name} : {kind:9s} sur 127.0.0.1:{p}')
            node.phone.on_notify.append(lambda u, d, node=node: [l.on_notify(u, d) for l in self.links if l.node is node])

    def poll(self):
        for key, _ in self.sel.select(timeout=0):
            what = key.data
            if what[0] == 'listen':
                conn, addr = key.fileobj.accept()
                link = Link(self, what[1], conn, what[2])
                self.links.append(link)
                self.sel.register(conn, selectors.EVENT_READ, ('link', link))
                print(f'[{what[1].name}] client {what[2]} connecté depuis {addr[0]}:{addr[1]}')
                link.start()
            else:
                link = what[1]
                try:
                    data = link.sock.recv(4096)
                except OSError:
                    data = b''
                if not data:
                    self.sel.unregister(link.sock)
                    link.sock.close()
                    self.links.remove(link)
                    print(f'[{link.node.name}] client {link.kind} déconnecté')
                    if link.kind == 'companion' and not any(l.node is link.node and l.kind == 'companion'
                                                             for l in self.links):
                        link.node.phone.disconnect()
                else:
                    link.feed(data)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('elf')
    ap.add_argument('-n', type=int, default=1, help='nombre de nœuds')
    ap.add_argument('--port', type=int, default=5000)
    ap.add_argument('--flash-dir')
    ap.add_argument('--speed', type=float, default=1.0)
    ap.add_argument('--gps', help='position GNSS « lat,lon[,alt] »')
    ap.add_argument('--battery', type=int, default=3900, help='tension batterie (mV)')
    ap.add_argument('--trace', action='store_true')
    a = ap.parse_args()

    def stop(sig, frm):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, signal.default_int_handler)

    w = World()
    for i in range(a.n):
        name = f'T1000A-{chr(65 + i)}'
        flash = None
        path = os.path.join(a.flash_dir, f'{name}.bin') if a.flash_dir else None
        if path and os.path.exists(path):
            flash = open(path, 'rb').read()
            print(f'{name} : Flash chargée depuis {path}')
        node = w.add(T1000A(a.elf, name=name, flash=flash, trace=a.trace, battery_mv=a.battery))
        if a.gps:
            v = [float(x) for x in a.gps.split(',')]
            node.gnss.fix(*v)
    print('Démarrage des jumeaux…', flush=True)
    w.run(3000)
    for node in w.nodes:
        print(f'  {node.name} annonce « {node.ble.adv_name()} »')
    br = Bridge(w, a.port)
    print('Prêt (Ctrl-C pour arrêter).', flush=True)
    t_real, t_virt = time.monotonic(), w.now_ms()
    step = 20
    try:
        while True:
            br.poll()
            w.run(step)
            for node in w.nodes:
                if node.tw.state == 'dfu':
                    print(f'[{node.name}] passé en DFU (bootloader) : arrêt de ce nœud')
                    node.tw.state = 'halted'
            if a.speed > 0:
                ahead = (w.now_ms() - t_virt) / 1000 / a.speed - (time.monotonic() - t_real)
                if ahead > 0:
                    time.sleep(ahead)
    except KeyboardInterrupt:
        print('\nArrêt.')
    finally:
        if a.flash_dir:
            os.makedirs(a.flash_dir, exist_ok=True)
            for node in w.nodes:
                p = os.path.join(a.flash_dir, f'{node.name}.bin')
                open(p, 'wb').write(node.flash_image())
                print(f'{node.name} : Flash enregistrée dans {p}')


if __name__ == '__main__':
    main()
