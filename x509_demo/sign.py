from M2Crypto import BIO, Rand, SMIME

def makebuf(text):
    return BIO.MemoryBuffer(text)

# Make a MemoryBuffer of the message.
buf = makebuf('a sign of our times'.encode())

# Seed the PRNG.
Rand.load_file('randpool.dat', -1)

# Instantiate an SMIME object; set it up; sign the buffer.
s = SMIME.SMIME()
s.load_key('signer_key.pem', 'signer.pem')
p7 = s.sign(buf, SMIME.PKCS7_DETACHED)

# Recreate buf.
buf = makebuf('a sign of our times'.encode())

# Output p7 in mail-friendly format.
out = BIO.MemoryBuffer()
out.write('From: sender@example.dom\n')
out.write('To: recipient@example.dom\n')
out.write('Subject: M2Crypto S/MIME testing\n')
s.write(out, p7, buf)

#print(out.read())
with open('sign.p7', 'w') as f:
    bytestring = out.read()
    print(bytestring)
    f.write(bytestring.decode())

# Save the PRNG's state.
Rand.save_file('randpool.dat')
