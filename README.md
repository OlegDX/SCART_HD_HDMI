# SCART_HD_HDMI
Reverse engineering

## STM8S003F3 - Restore in STMicroelectronics\st_toolset\stvd\stvdebug.exe:
```
address flash memory:
0x8000 - 0x8CE9
```

![](/STM8S003F3_1.png)
![](/STM8S003F3_2.png)


## STM8S003F3 - eprom update:
```
other:
  0x405d: 43 42 7B 00 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 4A 33 00 34 00 00 00 00
  0x40dd: FF

serial:
  0x4260: 00 2B 00 06 0E 47 35 33 31 38 37 33 00 00 00 00
  0x4270: DE 9B 38 D0 7E 57 69 AB 51 5E AF 5D 00 00 00 00
```
![](/eprom_1.png)
![](/eprom_2.png)
