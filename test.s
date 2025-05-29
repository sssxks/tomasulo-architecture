.global __start
__start:
addi x0, x0, 0
lw x2, 4(x0)
# x2=0x08

# 08 x4=0x10
lw x4, 8(x0)


# 0C x1=0x18
add x1, x2, x4


# 10 x1=0x17 overwrite
addi x1, x1, -1


# 14 x5=0x14
lw x5, 12(x0)


# 18 x6=0xffff0000
lw x6, 16(x0)


# 1C x7=0x0fff0000
lw x7, 20(x0)


# 20 x1=0x08
sub x1,x4,x2


# 24 x1=0xfffffffd
addi x1,x0,-3


# 28 not jump
beq  x4,x5,label0


# 2C jump
beq  x4,x4,label0


# 30
addi x12,x0,0x7FF


# 34
addi x12,x0,0x7FF


# 38
addi x12,x0,0x7FF


# 3C
addi x12,x0,0x7FF

label0:
# 40 x1=0x4000
lui  x1,4


# 44 jump x1=0x48
jal  x1,12


# 48
addi x12,x0,0x7FF


# 4C
addi x12,x0,0x7FF


# 50 x1=0xffff0050
auipc x1, 0xffff0


# 54 x8=0x1ffe000
div x8, x7, x2


# 58 x9=0x140
mul x9, x4, x5


# 5C x9=0xfff0000
mul x9, x8, x2


# 60 x2=0x04
addi x2, x0, 4


# 64
sw x8, 24(x0)


# 68 overwrite
sw x9, 24(x0)


# 6C x7=0xfff0000
lw x10, 24(x0)


# 70 x1=0x74
jalr x1,0(x0)