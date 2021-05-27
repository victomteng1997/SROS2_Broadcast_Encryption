from ecdsa import SigningKey, VerifyingKey
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
import binascii
import os
from M2Crypto import BIO, Rand, SMIME, X509
from x509_modules import x509_encrypt, x509_decrypt
from ecc_modules import ecc_encrypt, ecc_decrypt
import random
import hashlib
import pickle
import codecs


class EfficientBroadcastEncryption():
    def __init__(self):
        # TODO: load two sets of public key system
        self.g = 2

    def encrypt(self, M, S):
        ## 1. sig-gen
        sk = SigningKey.generate()  #
        vk = sk.verifying_key
        print(vk.to_string())

        ## 2. choose a random symmetric K.
        key = os.urandom(32)
        iv = os.urandom(16)
        final_key = key+iv
        aesCipher = Cipher(algorithms.AES(key), modes.CTR(iv))

        ## 3. Get T
        r = random.randint(1,64)
        T = self.g**r

        ## 4. for each pk in S, Cpk encrypt
        cpk_dic = {}
        for (pk, a) in S:
            # TODO: discuss on the methods to distribute a value. For each public key system, g**a should be known
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

    def decrypt(self, ciphertext):
        # parser
        ciphertext_byte = ciphertext.encode()
        [sigma, T, C1, C2] = pickle.loads(codecs.decode(ciphertext_byte, "base64"))
        m = hashlib.sha256()
        m.update(str(T**5).encode())
        hash_l = m.digest()
        C1_list = pickle.loads(C1)
        for _cj in C1_list:
            if hash_l in _cj:
                cj = _cj[len(hash_l):]
        if not cj:
            # message is not for this recipient. Return False
            return False
        try:
            p = ecc_decrypt('keyfiles/0_key.pem', cj)
        except:
            # decryption failed
            return False

        ## parse p as vk||x||K
        # TODO: for current implementation, len(vk) is 48 and len(K) = 32. Need to confirm later.
        vk_string = p[0:48]
        x = p[48:-48]
        K = p[-48:-16]
        iv = p[-16:]
        if not x == str(T**5).encode():
            print("T fail")
            return False
        try:
            vk = VerifyingKey.from_string(vk_string)
            assert vk.verify(sigma, str(T).encode() + C1 + C2)
            aesCipher = Cipher(algorithms.AES(K), modes.CTR(iv))
            Mdecryptor = aesCipher.decryptor()
            result = Mdecryptor.update(C2) + Mdecryptor .finalize()
            return result
        except: # assert error or final decrypt error.
            print("vk fail")
            return False




S = [('keyfiles/0_public.pem', 5),('keyfiles/1_public.pem', 6)] # test public key
M = b"message to encrypt"

EncryptionModel = EfficientBroadcastEncryption()
ciphertext = EncryptionModel.encrypt(M, S)
# print("ciphtertext:", ciphertext)


result = EncryptionModel.decrypt(ciphertext)

print(result)