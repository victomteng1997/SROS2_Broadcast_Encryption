from M2Crypto import BIO, Rand, SMIME, X509
from cryptography import x509

def makebuf(text):
    return BIO.MemoryBuffer(text)

def x509_encrypt(key_location, msg):
    if type(msg) == 'str':
        msg = msg.encode()
    buf = makebuf(msg)
    # Rand.load_file('randpool.dat', -1)
    s = SMIME.SMIME()

    # Load target cert to encrypt to.
    x509 = X509.load_cert(key_location)
    sk = X509.X509_Stack()
    sk.push(x509)
    s.set_x509_stack(sk)

    # Set cipher: 3-key triple-DES in CBC mode.
    s.set_cipher(SMIME.Cipher('des_ede3_cbc'))

    # Encrypt the buffer.
    p7 = s.encrypt(buf)

    # Output p7 in mail-friendly format.
    out = BIO.MemoryBuffer()
    out.write('From: sender@example.dom\n')
    out.write('To: recipient@example.dom\n')
    out.write('Subject: M2Crypto S/MIME testing\n')
    s.write(out, p7)

    bytestring = out.read()
    # print(bytestring)
    return bytestring

def x509_decrypt(public_key_location, secret_key_location, msg):
    # Instantiate an SMIME object.
    s = SMIME.SMIME()

    # Load private key and cert.
    s.load_key(secret_key_location, public_key_location)

    # Load the encrypted data.
    with open('encrypt.p7', 'w') as f:
        f.write(msg.decode())
    p7, data = SMIME.smime_load_pkcs7("encrypt.p7")
    # Decrypt p7.
    out = s.decrypt(p7).replace(b"\r\n",b"\n")

    return out
    # print(out)

def x509_sign(public_key_location, secret_key_location, message):
    buf = makebuf('a sign of our times'.encode())

    # Seed the PRNG.
    # Rand.load_file('randpool.dat', -1)

    # Instantiate an SMIME object; set it up; sign the buffer.
    s = SMIME.SMIME()
    s.load_key(secret_key_location, public_key_location)
    p7 = s.sign(buf, SMIME.PKCS7_DETACHED)

    # Recreate buf.
    buf = makebuf(message)

    # Output p7 in mail-friendly format.
    out = BIO.MemoryBuffer()
    s.write(out, p7, buf)

    bytestring = out.read()

    return bytestring

def x509_verify(public_key_location, msg):
    s = SMIME.SMIME()

    # Load the signer's cert.
    x509 = X509.load_cert(public_key_location)
    sk = X509.X509_Stack()
    sk.push(x509)
    s.set_x509_stack(sk)

    # Load the signer's CA cert. In this case, because the signer's
    # cert is self-signed, it is the signer's cert itself.
    st = X509.X509_Store()
    st.load_info(public_key_location)
    s.set_x509_store(st)

    # Load the data, verify it.
    with open('signed.p7', 'wb') as f:
        f.write(msg)
    p7, data = SMIME.smime_load_pkcs7('signed.p7')

    v = s.verify(p7, data)
    print(v)