#!/usr/bin/env python
# -*- coding: utf-8 -*-
wall = 16 #mm
p1 = [0,0]
p2 = [wall, 0]
p3 = [2*wall+483+235,0]
p4 = [p3[0]+wall,0]
p5 = [wall, wall]
p6 = [wall + 483, wall]
p7 = [p6[0]+wall, wall]
p8 = [p7[0]+235, wall]
p9 = [p8[0]+wall,wall]
p10 = [wall, wall + 245]
p11 = [wall + 295, wall + 245]
p12 = [wall + 295, wall + 245 - 90]
p13 = [p12[0] + wall, p12[1]]
p14 = [wall + 483, wall + 220]
p15 = [p14[0] + wall, p14[1]]
p16 = [p15[0] + 85, p15[1]]
p17 = [p16[0], p16[1] + wall]
p18 = [p14[0], p14[1] + wall]
p19 = [p13[0], p13[1] + 200]
p20 = [p12[0], p12[1] + 200]
p21 = [p11[0], p11[1] + wall]
p22 = [p10[0], p10[1] + wall]
p23 = [wall, 2* wall + 245 + 237]
p24 = [wall + 300, p23[1]]
p25 = [p24[0], p24[1] + wall]
p26 = [p23[0], p23[1] + wall]
p27 = [wall, 3* wall + 245 + 237+225]
p28 = [wall, p27[1] + wall]
p29 = [0, p27[1] + wall]
p30 = [p27[0] + 467, p27[1]]
p31 = [p30[0] + wall, p30[1]]
p32 = [p31[0] + 255, p31[1]]
p33 = [p32[0], p32[1] + wall]
p34 = [p33[0] + wall, p33[1]]
p35 = [p31[0], p31[1] - 220]
p36 = [p35[0] + 110 - wall - 45, p35[1]]
p37 = [p36[0], p36[1] - wall]
p38 = [p37[0] - 110, p37[1]]
p39 = [p38[0], p38[1] + wall]
p40 = [p39[0] + 45, p39[1]]
p41 = [wall + 300, wall*2 + 245 + 237]
p42 = [wall + 300, wall*3 + 245 + 237]
p43 = [wall + 300 + 220, wall*3 + 245 + 237]
p44 = [wall + 300 + 220, wall*2 + 245 + 237 - 5 - 40]
p45 = [p44[0] + wall, p44[1]]
p46 = [p44[0], p44[1] + 110]
p47 = [p46[0] + wall, p46[1]]
p48 = [wall + 300 + 220, wall*2 + 245 + 237] 
rects = [[p1,p29,p2,p28], [p2,p5,p3,p8],[p3,p33,p4,p34],[p27,p28,p32,p33], [p10,p22,p11,p21], [p12,p20,p13,p19], [p6,p14,p7,p15],[p14,p18,p16,p17],[p23,p26,p24,p25], [p48,p41,p43,p42], [p44, p46, p45, p47]]
 #, [p40,p30,p35,p31], [p38, p39, p37, p36]
