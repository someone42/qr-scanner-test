qr-scanner-test
===============

Proof-of-concept QR code scanner for STM32F4 discovery kit.

This code is based on the ZXing library. See https://code.google.com/p/zxing/ for
more information.

Hardware requirements
===============
The main board is the STM32F4 discovery kit.
See http://www.st.com/web/catalog/tools/FM116/SC959/SS1532/PF252419 for more information.

The main board needs to be attached to 3 of element14's extension boards.
See http://www.element14.com/community/docs/DOC-51084?ICID=knode-STM32F4-cameramore for more information.
The following three boards are required:
- "Base Board": a breakout board
- "Digital Camera Module": contains a OV9655 camera
- "LCD module": contains a 3.5" colour LCD

Software requirments
===============
The firmware was compiled with IAR Embedded Workbench for ARM 6.70.
The resulting binary is large (> 100 kB), so the lite version is not sufficient.
With some work, it's probably possible to get it to compile using other toolchains.

Known problems
===============
- Memory use is inefficient
- Sometimes the firmware hangs due to out-of-memory issues
- Finder pattern search could be optimised by using ARM Cortex bit-banding
- OV9655 camera is somewhat unsuitable due to its inability to focus on
  objects closer than about 1 foot
- Automatic exposure control gets confused by dark backgrounds and can end up
  overexposing a bright QR code over a dark background
- Viewfinder shows output of binarizer, which looks terrible
- Viewfinder frame rate is slower than what is probably possible
