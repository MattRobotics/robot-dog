// SCRATCH: CCOUNT per instruction under QEMU -icount, three instruction mixes.
#include <Arduino.h>
static inline uint32_t cc(){uint32_t v;__asm__ volatile("rsr %0, ccount":"=a"(v));return v;}
__attribute__((noinline)) int leaf(int a){return a*3+1;}
void setup(){
  Serial.begin(115200);delay(200);
  for(int rep=0;rep<3;rep++){
    portDISABLE_INTERRUPTS();
    uint32_t a=cc();
    __asm__ volatile("movi a8, 20000\n loop a8, 1f\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n 1:\n":::"a8");
    uint32_t b=cc();
    // ALU + taken branch, no zero-overhead loop: 2000 x (8 add + addi + bnez) = 20000 x 10 instructions
    __asm__ volatile("movi a9, 20000\n 2:\n add a10,a10,a9\n add a11,a11,a10\n add a10,a10,a11\n add a11,a11,a9\n add a10,a10,a11\n add a11,a11,a10\n add a10,a10,a9\n add a11,a11,a10\n addi a9,a9,-1\n bnez a9, 2b\n":::"a9","a10","a11");
    uint32_t c=cc();
    // loads/stores
    static volatile uint32_t buf[8];
    __asm__ volatile("movi a9, 20000\n 3:\n l32i a10,%0,0\n s32i a10,%0,4\n l32i a11,%0,8\n s32i a11,%0,12\n l32i a10,%0,16\n s32i a10,%0,20\n l32i a11,%0,24\n s32i a11,%0,28\n addi a9,a9,-1\n bnez a9, 3b\n"::"a"(buf):"a9","a10","a11","memory");
    uint32_t d=cc();
    portENABLE_INTERRUPTS();
    Serial.printf("QCAL2 nop_loop=%u alu_branch=%u load_store=%u per 200002/200001/200001 insns cpu_mhz=%u\n",(unsigned)(b-a),(unsigned)(c-b),(unsigned)(d-c),(unsigned)getCpuFrequencyMhz());
  }
  Serial.println("QCAL2 DONE");
}
void loop(){delay(1000);}
