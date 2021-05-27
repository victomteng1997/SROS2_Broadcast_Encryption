from ecies.utils import generate_eth_key
from ecies import encrypt, decrypt
import binascii
import os

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