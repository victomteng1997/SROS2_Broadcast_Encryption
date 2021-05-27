# SROS2_Broadcast_Encryption

## Intro
A broadcast encryption solution for ROS2 to address several existing vulnerabilities/weaknesses in current SROS2 implementation.
Following research work: https://www.adambarth.com/papers/2006/barth-boneh-waters.pdf

## Implementation
The current implementation uses X509 certificate as original ROS2 authentication method, ECDSA as broadcast encryption authentication method, and ECC as main encryption/decryption scheme. 
Will update the algorithms and documentations along development.

## To-dos
- [x] Development doc: https://victomteng1997.github.io/2021/05/20/ROS2-encryption-design/ 
- [ ] Update development doc (terms, algorithms, designs)
- [ ] Add requirements.txt
- [ ] Massive tests on different formats of messages
- [ ] ROS2 Integration: code integration
- [ ] ROS2 Integration: 
## Development Update
[27/05/2021] Basic framework is finished. Plan to update key management and distribution. 



