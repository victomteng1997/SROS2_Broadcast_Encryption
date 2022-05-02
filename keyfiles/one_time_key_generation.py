from ecies.utils import generate_eth_key
from ecies import encrypt, decrypt
import binascii
import os
import sys

def generate_keys(i):
    for i in range(0,3):
        privKey = generate_eth_key()
        privKeyHex = privKey.to_hex()
        pubKeyHex = privKey.public_key.to_hex()
        priv_key_file = str(i) + "_key.pem"
        public_key_file = str(i) + "_public.pem"
        with open(priv_key_file, 'w') as f:
            f.write(privKeyHex)
        with open(public_key_file, 'w') as f:
            f.write(pubKeyHex)


if __name__ == "__main__":
    arguments = sys.argv[1:]
    if len(arguments) == 2:
        i = arguments[1]
    else:
        i = 3
    generate_keys(i)
    
    with open("0_key.pem", 'r') as f:
        privKeyHex = f.read()

    with open("0_public.pem", 'r') as f:
        pubKeyHex = f.read()

    print("Encryption public key:", pubKeyHex)
    print("Decryption private key:", privKeyHex)

    plaintext = os.urandom(1000)
    print("Plaintext:", plaintext)

    encrypted = encrypt(pubKeyHex, plaintext)
    print("Encrypted:", binascii.hexlify(encrypted))

    decrypted = decrypt(privKeyHex, encrypted)
    print("Decrypted:", decrypted)
