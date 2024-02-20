import time
import socketserver
import threading
import itertools
import random


class SimTCPHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        print("[TCP Connected]")
        try:
            chunks = [tcp_data[i:i+8] for i in range(0, len(tcp_data), 8)]
            for d in itertools.cycle(chunks):
                self.wfile.write(d)
                time.sleep(0.1)

                if random.randint(0, 100) == 0:
                    # Write some random data to simulate a missalignment error
                    self.wfile.write(random.randbytes(random.randint(1, 8)))

        except Exception as e:
            print(e)
            pass


class SimUDPHandler(socketserver.DatagramRequestHandler):
    def handle(self) -> None:
        data = self.rfile.read()
        hex = data.hex(" ").upper()
        print(f"[UDP Data]    {hex}")

        # Update the flag part of header to reflect that this is an acknowledge package
        data = bytearray(data)
        if len(data) >= 8:
            data[1] = (data[1] & 0xF0) | (0b1000)

        self.wfile.write(data)


if __name__ == "__main__":
    tcp_data = None
    with open("sim_tcp_data.bin", "rb") as f:
        tcp_data = f.read()

    HOST = "0.0.0.0"
    TCP_PORT = 8765
    UDP_PORT = 7654

    tcp_server = socketserver.ThreadingTCPServer((HOST, TCP_PORT), SimTCPHandler)
    udp_server = socketserver.ThreadingUDPServer((HOST, UDP_PORT), SimUDPHandler)

    with tcp_server, udp_server:
        print(f"TCP on port {tcp_server.server_address[1]}")
        print(f"UDP on port {udp_server.server_address[1]}")
        print("..................................................................")

        threads = [
            threading.Thread(target=tcp_server.serve_forever),
            threading.Thread(target=udp_server.serve_forever),
        ]

        for thread in threads:
            thread.daemon = True
            thread.start()

        try:
            while True:
                time.sleep(1)
        except:
            tcp_server.shutdown()
            udp_server.shutdown()
            for thread in threads:
                thread.join()
            tcp_server.server_close()
            udp_server.server_close()


