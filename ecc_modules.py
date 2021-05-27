from ecies.utils import generate_eth_key
from ecies import encrypt, decrypt
import binascii
import os

def ecc_encrypt(public_key_location, message):
    with open(public_key_location, 'r') as f:
        pubKeyHex = f.read()
    encrypted = encrypt(pubKeyHex, message)
    return encrypted


def ecc_decrypt(private_key_location, cipher):
    with open(private_key_location, 'r') as f:
        privKeyHex = f.read()
    decrypted = decrypt(privKeyHex, cipher)
    return decrypted