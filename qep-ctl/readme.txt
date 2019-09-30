---------
qep-ctl
---------
A utility to configure QDMA Ethernet Platform (QEP). 

Build:
-------
	gcc qep_ctl.c -o qep-ctl

Supported Command:
------------------
qep-ctl supports following commands.

	1. help : This command displays a short usage. The output looks as below.
		./qep-ctl CMD [ options ]
		  CMD = [ config | show | read | write | help ]
		  Options =  -d BDF
					 -b bar_num
					 -c [ tx | rx ]
					 -m mac_addr
					 -v VID,DEI,PCP
					 -e [ mac,vlan,discard ] [ disable ]
	2. config : This command configures the Ethernet MAC address and 802.1Q 
	VLAN fields. 
		-m: With this option Ethernet MAC address is updated in the hardware.
			The MAC address is used by qep drivers at probe time. 
			e.g. ./qep-ctl config -m 01:02:03:03:04:06

		-v: VLAN identifier(VID), Drop eligible indicator(DEI)and Priority code
			point(PCP) are provided with (-v) option. The values are comma 
			separated in order of VID, DEI, PCP respectively. DEI and PCP are 
			optional. 
			e.g. ./qep-ctl config -v 234,0,3
		
		-e: CLI supports upto four inputs with (-e) option. 
			->'mac': This enables MAC update in tx direction. The source MAC 
				address ofall outgoing packets will be updated with programmed 
				MAC address.It also enables MAC filtering in rx direction. 
				Incoming packets whose destination MAC address matches the  
				programmed MAC will only be accepted.
			->'vlan': This enables VLAN tag insertion in tx direction. For all 
				outgoing packets, a standard 802.1q VLAN TAG is inserted with 
				programmed values. It also enables VLAN ID based packet filtering
				in rx direction. The device only accepts an incoming packet if 
				it contains a valid VLAN tag and VLAN ID matches with programed 
				VLAN ID.
			->'discard': This input drops all outgoing packets for tx and all 
				incoming packets for rx.
			->'disable': This input disables the security block and clears all
				enable flags.
		Comma separated list of values can be provided for enabling multiple 
		features and direction can sleceted using general option.
		e.g.  ./qep-ctl config -e mac,vlan -c tx
		
	3. show: This command displays the current configuration of the security 
		block.
		e.g.  ./qep-ctl show
	
	4 read: This command is used to read a register directly. The offset must be
		the first parameter to read command. By default, it uses first found 
		device and user BAR i.e bar number 2. To read from different device 
		and BAR use general options.
		e.g.  ./qep-ctl read 0x0010000
	
	5. write: This command is used to write to a register directly. It 
		takes offset and value as next two parameters. By default, it uses first
		found device and user BAR i.e bar number 2. To write to different device
		and BAR use general options.
		e.g.  ./qep-ctl write 0x0010000 0xef
	
General Options:
----------------
qep-ctl provides below general options. 

 -d: This option is used for device selection and must be followed by valid 
	BDF. If (-d) option is not provided, CLI looks for BDF of Xilinx PCI device 
	(0x7000). If the device has a different PCI device ID then PCIE_DEV_ID_MGMT_PF 
	macro in source code must be updated to detect the device. In case, the 
	system has multiple devices, then BDF must be provided with each command. 

 -b: This option is used to specify PCIe BAR number for direct register read
	and write. By default, BAR number is 2 which is user BAR. Default user bar
	can be updated using DEFAULT_USER_BAR macro in source code.

 -c: This option is used to specify the direction. It takes 'rx' or 'tx' as a 
	parameter. if this option is not specified command is applied on both 
	directions.
