from ecdsa import SigningKey, VerifyingKey
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.backends import default_backend
import binascii
import os
#from M2Crypto import BIO, Rand, SMIME, X509
#from x509_modules import x509_encrypt, x509_decrypt
from ecc_modules import ecc_encrypt, ecc_decrypt
import random
import string
import hashlib
import pickle
import codecs
import sys



class EfficientBroadcastEncryption():
    def __init__(self):
        self.g = 2
        self.a = 5

    def encrypt(self, M, S):
        ## 1. sig-gen
        sk = SigningKey.generate()  #
        vk = sk.verifying_key
        #print(vk.to_string())

        ## 2. choose a random symmetric K.
        key = os.urandom(32)
        iv = os.urandom(16)
        final_key = key+iv
        aesCipher = Cipher(algorithms.AES(key), modes.CTR(iv), default_backend())

        ## 3. Get T
        r = random.randint(1,64)
        T = self.g**r

        ## 4. for each pk in S, Cpk encrypt
        cpk_dic = {}
        for (pk, a) in S:
            print("encrypting for receivers:", pk)
            gar = str(self.g**(a*r)).encode()
            # print('gar:', gar)
            m = hashlib.sha256()
            m.update(gar)
            hash_gar = m.digest()
            vgf = vk.to_string() + gar + final_key
            cpk = hash_gar + ecc_encrypt(pk, vgf)
            # print("cpk: ", cpk)
            cpk_dic[hash_gar] = cpk
        # 5.
        cpk_list = []
        for key in sorted(cpk_dic):
            cpk_list.append(cpk_dic[key])
        #C1 = b''.join(cpk_list)
        C1 = pickle.dumps(cpk_list) # C1 in bytes

        # 6.
        encryptor = aesCipher.encryptor()
        C2 = encryptor.update(M) + encryptor.finalize()

        # 7. Sign
        sigma = sk.sign(str(T).encode() + C1 + C2)
        assert vk.verify(sigma, str(T).encode() + C1 + C2)

        ciphertext = codecs.encode(pickle.dumps([sigma, T, C1, C2]), "base64").decode()

        return ciphertext

    def decrypt(self, ciphertext, private_key):
        # parser
        ciphertext_byte = ciphertext.encode()
        pk_file = private_key[0]
        a = private_key[1]
        [sigma, T, C1, C2] = pickle.loads(codecs.decode(ciphertext_byte, "base64"))
        m = hashlib.sha256()
        m.update(str(T**a).encode())
        hash_l = m.digest()
        C1_list = pickle.loads(C1)
        cj = False
        for _cj in C1_list:
            if hash_l in _cj:
                cj = _cj[len(hash_l):]
        if not cj:
            # message is not for this recipient. Return False
            print("message is not for", pk_file)
            return False
        try:
            p = ecc_decrypt(pk_file, cj)
        except:
            # decryption failed
            return False

        ## parse p as vk||x||K
        vk_string = p[0:48]
        x = p[48:-48]
        K = p[-48:-16]
        iv = p[-16:]
        if not x == str(T**a).encode():
            #print("T fail")
            return False
        try:
            vk = VerifyingKey.from_string(vk_string)
            assert vk.verify(sigma, str(T).encode() + C1 + C2)
            aesCipher = Cipher(algorithms.AES(K), modes.CTR(iv), default_backend())
            Mdecryptor = aesCipher.decryptor()
            result = Mdecryptor.update(C2) + Mdecryptor .finalize()
            return result
        except: # assert error or final decrypt error.
            #print("vk fail")
            return False


def get_random_string(length):
    # choose from all lowercase letter
    letters = string.ascii_lowercase
    result_str = ''.join(random.choice(letters) for i in range(length))
    return result_str
    
def cipher_plain_ratio_unit_test():
    S = [('keyfiles/0_public.pem', 5),('keyfiles/1_public.pem', 6)] # test public key
    for i in range(1,256):
        M = str.encode(get_random_string(i))
        EncryptionModel = EfficientBroadcastEncryption()
        ciphertext = EncryptionModel.encrypt(M, S)
        result = EncryptionModel.decrypt(ciphertext)
        #print(len(ciphertext), i)
        
        
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
