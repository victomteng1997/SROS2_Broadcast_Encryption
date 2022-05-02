from ecc_modules import ecc_encrypt, ecc_decrypt
import random
import string
import hashlib
import pickle
import codecs
import sys
from efficient_encryption import EfficientBroadcastEncryption


if __name__ == "__main__":
    #cipher_plain_ratio_unit_test()
    S = [('keyfiles/0_public.pem', 5),('keyfiles/1_public.pem', 6)] # public keys. Noted that each key should be binded with a pre-defined fix value "a", distributed together with the key. If not provided, a default value will be used.
    Private_Keys = [('keyfiles/0_key.pem', 5),('keyfiles/1_key.pem', 6)] # private keys.
    
    arguments = sys.argv[1:]
    if len(arguments) > 0:
        M = arguments[0].encode()
    else:
        M = b"message_to_encrypt"
    
    EncryptionModel = EfficientBroadcastEncryption()
    ciphertext = EncryptionModel.encrypt(M, S)
    for private_key in Private_Keys:
        print("decrypting message with private_key:", private_key[0])
        result = EncryptionModel.decrypt(ciphertext, private_key)
        print(result)
