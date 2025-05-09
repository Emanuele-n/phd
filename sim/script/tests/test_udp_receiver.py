import socket
import struct

# Configuration
UDP_IP = "192.168.101.148"
UDP_PORT = 25000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening on {UDP_IP}:{UDP_PORT}")

try:
    while True:
        data, addr = sock.recvfrom(1024)  # Adjust buffer size if necessary
        
        if len(data) == 4:  # Likely a single precision float or 32-bit integer
            try:
                # Attempt to decode as a single precision float
                value_float = struct.unpack('f', data)
                print(f"Received float: {value_float[0]} from {addr}")
            except:
                pass

            try:
                # Attempt to decode as a 32-bit integer
                value_int = struct.unpack('i', data)
                print(f"Received int: {value_int[0]} from {addr}")
            except:
                pass

        elif len(data) == 8:  # Likely a double precision float or 64-bit integer
            try:
                # Attempt to decode as a double precision float
                value_double = struct.unpack('d', data)
                print(f"Received double: {value_double[0]} from {addr}")
            except:
                pass

        # Example of decoding a fixed-length string (adjust length as needed)
        elif len(data) > 0:  # Assuming there's no fixed size, trying to interpret as a string
            try:
                # Decode as UTF-8 string, replace errors to avoid exceptions
                value_str = data.decode('utf-8', errors='replace')
                print(f"Received string: {value_str} from {addr}")
            except:
                print(f"Received unhandled data type: {data} from {addr}")

except KeyboardInterrupt:
    print("Stopping receiver.")
    sock.close()
