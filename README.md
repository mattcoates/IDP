# IDP Team 2 - Lent 2017

## Build for Workstation
- $ make all

### Department SSH Login Instructions
- $ ssh -X mc955@gate.eng.cam.ac.uk
- $ slogin -X ts-access

### Upload Builds to Department Folder
- $ scp 'file-name' mc955@gate.eng.cam.ac.uk:/homes/mc955/idp_shared/mc955/'file-name'

### Robot Upload Instructions
- $ scp 'program-name'.arm team@wlan-robot15.private:'program-name'.arm
- $ Dirac

### Robot Login Instructions
- $ slogin team@wlan-robot15.private
- $ Dirac

#### Exceute Program:
- $ ./'program-name'.arm
