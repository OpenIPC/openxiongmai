# xm_v200_log

`xm_v200_log` is the V200 series' media log, at `/dev/log` and `/proc/umap/logmpp`, and builds as `open_log.ko`. It replaces the vendor `log.ko`.

`/dev/log` is a 32 KB FIFO of text: writes append, dropping the oldest when it is full, and reads wait for data. Each media module (sys, vb, ai, ao, aenc, adec, rgn, vi, vo, venc, vdec, vpss, var) has a log level from 1 to 7, 3 to start. `/proc/umap/logmpp` shows the levels and takes `<module> <level>`, and ioctl `0x800c4c00` returns the table.
