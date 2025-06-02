# CycloneDDS Communication
You need to create the CycloneDDS headers and core files before building  
```
cd dds_comm
mkdir generated && cd generated
idlc -l cxx ../idl/odom_pcl.idl
```
