# SROR2 Broadcast Encryption Implementation
This project includes the source code implementation for SROS2 Broadcast Encryption, submitted to ACMCCS 2022. It includes an easy-to-use script that demonstrates the usage of the solution. For full integration into SROS2, the solution should be imported as a package.
The project is only tested on Ubuntu 18.04/20.04 system. It has no specific hardware requirements, but for the integration of ROS2, please follow the hardware/system requirement as declared by ROS2 official website: https://docs.ros.org/en/crystal/Installation/Linux-Install-Binary.html

## Installation
1. Install the python dev environment: `apt-get install build-essential libssl-dev swig python3-dev`
2. Install dependencies: `pip3 install -r requirements.txt`



## Quickstart
The solution can be directly used without the presence of SROS2. To quickly start, you can use the pre-created key pairs from `keyfiles` folder with `python3 demo1.py  <your message>`
Ideally, the output should be :
```
$ python3 efficient_encryption.py  123
encrypting for receivers: keyfiles/0_public.pem
encrypting for receivers: keyfiles/1_public.pem
decrypting message with private_key: keyfiles/0_key.pem
b'123'
decrypting message with private_key: keyfiles/1_key.pem
b'123'
```
The above script encrypts the user-provided messages with the public key of the user "0" and "1", and then perform the decryption with their secret keys. 

Another example demonstrates that users cannot decrypt the message unless the publisher specifies it as the receiver: `python3 demo2.py <your message`. This example encrypts only with the public key of user 0, but both user 0 and user 1 try to decyprt it. The output should be.
```
$ python3 demo_2.py 123
encrypting for receivers: keyfiles/0_public.pem
decrypting message with private_key: keyfiles/0_key.pem
b'123'
decrypting message with private_key: keyfiles/1_key.pem
message is not for keyfiles/1_key.pem
False
```

## General Usage
As shown in `demo_1.py`, our solution is easy to use. 

The `EfficientBroadcastEncryption()` is the main class to encrypt/decrypt messages. User initialize the encryption/decryption object with `EncryptionModel = EfficientBroadcastEncryption()`.

To encrypt a message `M` and send to receivers `S`, the easy way is to `ciphertext = EncryptionModel.encrypt(M, S)`. `S` is in a format of `[(pk_file, a),(pk_file, b)]`, where `a,b,...` are a series of arbitrary integers bounded to each key pairs. 

To decryt the message, just use `EncryptionModel.decrypt(ciphertext, private_key)` to decrypt. 

## Key Management
We also provide easy-to-use script for the generation of keypairs. Our script `keyfiles/one_time_key_generation.py` helps to do this.

`python3 keyfiles/one_time_key_generation.py <n>` generates `n` keypairs under the `keyfiles` directory. You will see some encryption/decryption test output that verifies the generated keys are valid. By default, `n` is 3. You can use these keyfiles directly.

## SROS2 Integration.
Users should firstly install ROS2 following the official instructions. For Ubuntu system, you can follow: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html. This process will also install SROS2 automatically. 

A sample Python-based publisher/subscriber code can be found at: https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html 
The message publisher part looks like:
```
def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

```
The subscriber part:
```
def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
```

To add our solution, you can easily import the libraries, and do the encryption/decryption before/after the message communication. Publishing part:
```
ciphertext = EncryptionModel.encrypt(str.encode(rawdata), S)
msg.data = ciphertext
self.publisher_.publish(self.msg)
```
Subscription part:

```
result = EncryptionModel.decrypt(data.data)
```


