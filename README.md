# nbd-decoder
ネオジオCD上のBIOSを音声信号に変調して出力する
[ncd-bios-dumper](https://github.com/scrap-a/ncd-bios-dumper)
用のデコーダー。詳細はそちらを参照のこと。

## 実行方法

```
nbd_decoder.exe -m [mode] [in.wav] [out.bin]
  mode:am_mono_high am_mono_low
   (default) am_mono_high
```
