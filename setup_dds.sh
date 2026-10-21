#!/bin/bash
# Create CycloneDDS configuration
cat << 'XML' > ~/cyclonedds.xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
    <Domain id="any">
        <General>
            <Interfaces>
                <NetworkInterface name="wlp2s0"/>
            </Interfaces>
            <AllowMulticast>true</AllowMulticast>
        </General>
    </Domain>
</CycloneDDS>
XML

# Update .bashrc
sed -i '/CYCLONEDDS_URI/d' ~/.bashrc
sed -i '/RMW_IMPLEMENTATION/d' ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
echo 'export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml' >> ~/.bashrc
source ~/.bashrc
echo "CycloneDDS Wi-Fi setup complete!"
