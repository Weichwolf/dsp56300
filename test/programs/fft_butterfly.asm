; SPDX-License-Identifier: GPL-3.0-only
; Project-authored radix-2 DIF butterfly. No manufacturer source is reused.
; X/Y data planes: r0/r4 left, r1/r5 right, r2/r6 twiddles.
; Each stage rounds half sums/differences, then rounds the rotated difference.
        org p:$100
        move x:(r0),x0 y:(r4),y0
        move x:(r1),x1 y:(r5),y1
        tfr x0,a
        add x1,a
        tfr y0,b
        add y1,b
        asr a
        asr b
        rnd a
        rnd b
        move a,x:(r0)+ b,y:(r4)+
        tfr x0,a
        sub x1,a
        tfr y0,b
        sub y1,b
        asr a
        asr b
        rnd a
        rnd b
        move a,x0
        move b,y0
        move x:(r2)+n2,x1 y:(r6)+n6,y1
        mpy x0,x1,a
        mac -y0,y1,a
        mpy x0,y1,b
        mac y0,x1,b
        rnd a
        rnd b
        move a,x:(r1)+ b,y:(r5)+
        end
