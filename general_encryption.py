from ecdsa import SigningKey
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
import binascii
import os
from M2Crypto import BIO, Rand, SMIME, X509
from x509_modules import x509_encrypt, x509_decrypt
import random

# Define the block length constants. Use to parse the message


# Setup
"""
need to get initialization variables for signature algorithm
"""

# Keygen (by ROS2 Default)


# Encrypt (S: recipient, M: message)
S = ['x509_demo/recipient.pem'] # test public key
M = b"message to encrypt"
## 1. sig-gen
sk = SigningKey.generate() #
vk = sk.verifying_key
# print(vk.to_string())
# print(vk)

## 2. choose a random symmetric K.
key = os.urandom(32)
iv = os.urandom(16)
aesCipher = Cipher(algorithms.AES(key), modes.CTR(iv))

## 3. for each pk in S, Cpk encrypt
cpk_list = []
for pk in S:
    cpk = x509_encrypt(pk, vk.to_string()+key)
    print("cpk: ", cpk)
    cpk_list.append(cpk)
## 4. randomlize cpk_list and concat
random.shuffle(cpk_list)
C1 = b''.join(cpk_list)
#print("C1:", C1)
print(len(C1))

## 5. C2 = Ek(M)
encryptor = aesCipher.encryptor()
C2 = encryptor.update(M) + encryptor.finalize()
#print("C2:", C2)
print(len(C2))

### Sample decryption
"""
decryptor = aesCipher.decryptor()
result = decryptor.update(C2) + decryptor.finalize()
print(result)
"""

## 6. Sign with sk

sigma = sk.sign(C1+C2)
assert vk.verify(sigma, C1+C2)

## 7. Return ciphertext
ciphertext = sigma+C1+C2
print("Ciphertext from Encryption: ", ciphertext)


#############################################
# Decrypt (sk, C)

## TODO: add parsing. Need to know the size of each block in C1
try:
    p = x509_decrypt('x509_demo/recipient.pem', 'x509_demo/recipient_key.pem', C1)
except:
    # decryption failed
    pass
print(p)
## for this case, length of vk is 48
decrypted_vk = p[0:48]
decrypted_K = p[48:]

try:
    assert vk.verify(sigma, C1+C2)
    Mdecryptor = aesCipher.decryptor()
    result = Mdecryptor.update(C2) + Mdecryptor .finalize()
    print(result)
    assert result == M
except:
    print("Fail at final signature verification")
