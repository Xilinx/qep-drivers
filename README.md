# Xilinx QDMA Ethernet Platform (QEP) drivers

## QDMA Ethernet Platform

The design adds Ethernet support to QDMA based streaming platform.
The Ethernet Subsystem is added to the static region of the shell.
The platform has three physical functions, two physical functions for device management (PF0) and compute acceleration (PF1), and one physical function (PF2) for Network acceleration.
The Ethernet subsystem is accessible to the host via PF2.

Both the linux kernel driver and the DPDK driver can be run on a PCI Express root port host PC to interact with the QEP endpoint IP via PCI Express.
