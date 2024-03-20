/*
 * Copyright (c) 2021 The Regents of the University of California
 * Copyright (c) 2023 Google LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "arch/riscv/spmp.hh"
#include "arch/generic/tlb.hh"
#include "arch/riscv/faults.hh"
#include "arch/riscv/isa.hh"
#include "arch/riscv/regs/misc.hh"
#include "base/addr_range.hh"
#include "base/types.hh"
#include "cpu/thread_context.hh"
#include "base/trace.hh"
#include "debug/SPMPCheck.hh"
#include "debug/SPMP.hh"
#include "debug/PMP.hh"
#include "math.h"
#include "mem/request.hh"
#include "params/SPMP.hh"
#include "sim/sim_object.hh"


namespace gem5
{

SPMP::SPMP(const Params &params):
    SimObject(params),
    spmpEntries(params.spmp_entries),
    numRules(0)
{
    spmpTable.resize(spmpEntries);
}

Fault
SPMP::spmpCheck(RiscvISA::STATUS status, const RequestPtr &req, BaseMMU::Mode mode,
              RiscvISA::PrivilegeMode pmode, ThreadContext *tc,
              Addr vaddr)
{
    // 若shouldCheckSPMP返回值为0，代表不用进行check直接返回 当没有Spmp Entry或者当前运行模式为Machine mode时直接跳过check
    if (!shouldCheckSPMP(pmode, tc))
        return NoFault;

   

    // 利用match_index指示给定地址所对应的spmp entry
    int match_index = -1;


    if (numRules == 0 || (pmode == RiscvISA::PrivilegeMode::PRV_M))
         return NoFault;
    // spmp entry从低到高进行比对（低编号entry优先级高）
    for (int i = 0; i < spmpTable.size(); i++) {
        AddrRange spmp_range = spmpTable[i].spmpAddr; 
        if (spmp_range.contains(req->getPaddr()) &&//在spmptable中，每个entry有一个addr区间，该区间由配置pmp寄存器时 a字段的值及地址确定
                spmp_range.contains(req->getPaddr() + req->getSize() - 1)) {
            // 当paddr及paddr+size-1在entry地址范围内布，给match_index赋值
            match_index = i;
        }

         if ((match_index > -1)//spmp匹配规则
            && (SPMP_OFF != spmpGetAField(spmpTable[match_index].spmpCfg))) {
            uint8_t this_cfg = spmpTable[match_index].spmpCfg;

            if(((mode == BaseMMU::Mode::Write)||(mode == BaseMMU::Mode::Read))&&
                                    (~this_cfg&SPMP_S)&&(~this_cfg&SPMP_READ)&&(this_cfg&SPMP_WRITE)&&(this_cfg&SPMP_EXEC))//shr RW SRWX=0011
                return NoFault;

            if((pmode == RiscvISA::PrivilegeMode::PRV_S)&&
                                    ((mode == BaseMMU::Mode::Execute)||(mode == BaseMMU::Mode::Read))&&
                                        (this_cfg&SPMP_S)&&(~this_cfg&SPMP_READ)&&(this_cfg&SPMP_WRITE)&&(this_cfg&SPMP_EXEC))//shr RX SRWX=1011
                return NoFault;

            if((pmode == RiscvISA::PrivilegeMode::PRV_U)&&
                                    (mode == BaseMMU::Mode::Execute)&&
                                        (this_cfg&SPMP_S)&&(~this_cfg&SPMP_READ)&&(this_cfg&SPMP_WRITE)&&(this_cfg&SPMP_EXEC))//shr X SRWX=1011
                return NoFault;

            if((pmode == RiscvISA::PrivilegeMode::PRV_S)&&
                                    (mode == BaseMMU::Mode::Read)&&(mode == BaseMMU::Mode::Write)&&
                                        (~this_cfg&SPMP_S)&&(~this_cfg&SPMP_READ)&&(this_cfg&SPMP_WRITE)&&(~this_cfg&SPMP_EXEC))//shr RW SRWX=0010
                return NoFault;           

            if((pmode == RiscvISA::PrivilegeMode::PRV_U)&&
                                    (mode == BaseMMU::Mode::Read)&&
                                        (~this_cfg&SPMP_S)&&(~this_cfg&SPMP_READ)&&(this_cfg&SPMP_WRITE)&&(~this_cfg&SPMP_EXEC))//shr RO SRWX=0010
                return NoFault;

            if((mode == BaseMMU::Mode::Execute)&&
                                        (this_cfg&SPMP_S)&&(~this_cfg&SPMP_READ)&&(this_cfg&SPMP_WRITE)&&(~this_cfg&SPMP_EXEC))//shr X SRWX=1010
                return NoFault;
            
            if((pmode == RiscvISA::PrivilegeMode::PRV_S)&&
                                                (status.sum==1)&&
                                                        (~this_cfg&SPMP_S)){//S-mode s=0 status.sum=1 没有执行权限，根据读写权限位进行读写
                if((mode == BaseMMU::Mode::Read)&&(this_cfg&SPMP_READ))
                    return NoFault;

                else if((mode == BaseMMU::Mode::Write)&&(this_cfg&SPMP_WRITE))
                    return NoFault;

                else if (req->hasVaddr()) 
                    return createAddrfault(req->getVaddr(), mode);

                else 
                    return createAddrfault(vaddr, mode);
            }

            if(pmode == RiscvISA::PrivilegeMode::PRV_U){//U-mode s=0 根据读写执行权限位进行读写执行
                if((mode == BaseMMU::Mode::Read)&&(this_cfg&SPMP_READ))
                    return NoFault;
                else if((mode == BaseMMU::Mode::Write)&&(this_cfg&SPMP_WRITE))
                    return NoFault;
                else if((mode == BaseMMU::Mode::Execute)&&(this_cfg&SPMP_EXEC))
                    return NoFault;
                else if (req->hasVaddr()) 
                    return createAddrfault(req->getVaddr(), mode);
                else 
                    return createAddrfault(vaddr, mode);
            }
            if((pmode == RiscvISA::PrivilegeMode::PRV_S)&&
                                            (this_cfg&SPMP_S)){//S-mode且S=1 按照执行权限来执行
                if((mode == BaseMMU::Mode::Read)&&(this_cfg&SPMP_READ))
                    return NoFault;
                else if((mode == BaseMMU::Mode::Write)&&(this_cfg&SPMP_WRITE))
                    return NoFault;
                else if((mode == BaseMMU::Mode::Execute)&&(this_cfg&SPMP_EXEC))
                    return NoFault;
                else if (req->hasVaddr()) 
                    return createAddrfault(req->getVaddr(), mode);
                else 
                    return createAddrfault(vaddr, mode);
            }        
            if(((pmode == RiscvISA::PrivilegeMode::PRV_S)&&(status.sum==0)&&(~this_cfg&SPMP_S))||
                                                            ((pmode == RiscvISA::PrivilegeMode::PRV_U)&&(this_cfg&SPMP_S))){
            //当S-mode、S=0、status.sum=0时或者U-mode、S=1拒绝访问
                if (req->hasVaddr()) 
                    return createAddrfault(req->getVaddr(), mode);
                else 
                    return createAddrfault(vaddr, mode);
        }
        }
    }
    // if no entry matched and we are not in M mode return fault
    if (pmode == RiscvISA::PrivilegeMode::PRV_M) {
        return NoFault;
    } else if (req->hasVaddr()) {
        return createAddrfault(req->getVaddr(), mode);
    } else {
        return createAddrfault(vaddr, mode);
    }
}

Fault
SPMP::createAddrfault(Addr vaddr, BaseMMU::Mode mode)
{
    RiscvISA::ExceptionCode code;
    if (mode == BaseMMU::Read) {
        code = RiscvISA::ExceptionCode::LOAD_ACCESS;
    } else if (mode == BaseMMU::Write) {
        code = RiscvISA::ExceptionCode::STORE_ACCESS;
    } else {
        code = RiscvISA::ExceptionCode::INST_ACCESS;
    }
    warn("spmp access fault.\n");
    return std::make_shared<RiscvISA::AddressFault>(vaddr, code);
}

inline uint8_t
SPMP::spmpGetAField(uint8_t cfg)
{
    // to get a field from spmpcfg register
    uint8_t a = cfg >> 3;
    return a & 0x03;
}


bool
SPMP::spmpUpdateCfg(uint32_t spmp_index, uint8_t this_cfg)
{
    if (spmp_index >= spmpEntries) {
        DPRINTF(SPMP, "Can't update spmp entry config %u"
                " because the index exceed the size of spmp entries %u",
                spmp_index, spmpEntries);
        return false;
    }

    DPRINTF(SPMP, "Update spmp config with %u for spmp entry: %u \n",
                                    (unsigned)this_cfg, spmp_index);
    spmpTable[spmp_index].spmpCfg = this_cfg;
    spmpUpdateRule(spmp_index);
    return true;
}

void
SPMP::spmpUpdateRule(uint32_t spmp_index)
{
    // In qemu, the rule is updated whenever
    // spmpaddr/spmpcfg is written

    numRules = 0;
    Addr prevAddr = 0;

    if (spmp_index >= 1) {
        prevAddr = spmpTable[spmp_index - 1].rawAddr;
    }

    Addr this_addr = spmpTable[spmp_index].rawAddr;
    uint8_t this_cfg = spmpTable[spmp_index].spmpCfg;
    AddrRange this_range;

    switch (spmpGetAField(this_cfg)) {
      // checking the address matching mode of spmp entry
      case SPMP_OFF:
        // null region (spmp disabled)
        this_range = AddrRange(0, 0);
        break;
      case SPMP_TOR:
        // top of range mode
        this_range = AddrRange(prevAddr << 2, (this_addr << 2));
        break;
      case SPMP_NA4:
        // naturally aligned four byte region
        this_range = AddrRange(this_addr << 2, ((this_addr << 2) + 4));
        break;
      case SPMP_NAPOT:
        // naturally aligned power of two region, >= 8 bytes
        this_range = AddrRange(spmpDecodeNapot(this_addr));
        break;
      default:
        this_range = AddrRange(0,0);
    }

    spmpTable[spmp_index].spmpAddr = this_range;

    for (int i = 0; i < spmpEntries; i++) {
        const uint8_t a_field = spmpGetAField(spmpTable[i].spmpCfg);
      if (SPMP_OFF != a_field) {
          numRules++;
      }
    }

}

void
SPMP::spmpReset()
{
    for (uint32_t i = 0; i < spmpTable.size(); i++) {
        spmpTable[i].spmpCfg &= ~(SPMP_A_MASK);
        spmpUpdateRule(i);
    }
}

bool
SPMP::spmpUpdateAddr(uint32_t spmp_index, Addr this_addr)
{
    if (spmp_index >= spmpEntries) {
        DPRINTF(SPMP, "Can't update spmp entry address %u"
                " because the index exceed the size of spmp entries %u",
                spmp_index, spmpEntries);
        return false;
    }

    DPRINTF(SPMP, "Update spmp addr %#x for spmp entry %u \n",
                                      (this_addr << 2), spmp_index);

    // just writing the raw addr in the spmp table
    // will convert it into a range, once cfg
    // reg is written
    spmpTable[spmp_index].rawAddr = this_addr;
    for (int index = 0; index < spmpEntries; index++) {
        spmpUpdateRule(index);
    }

    return true;
}

bool
SPMP::shouldCheckSPMP(RiscvISA::PrivilegeMode pmode, ThreadContext *tc)
{
    //判断是否需要进行SPMP CHECK  返回1表示需要进行检查 返回0表示不需要进行检查 1、有lock必须检查（M模式也需要检查）2、没有lock 非M模式进行检查 
    //3、numRules==0不需要进行检查（numRules表示spmp entry有效的个数）
    return (numRules != 0) && (pmode != RiscvISA::PrivilegeMode::PRV_M);
}

AddrRange
SPMP::spmpDecodeNapot(Addr spmpaddr)
{
    if (spmpaddr == -1) {
        AddrRange this_range(0, -1);
        return this_range;
    } else {
        uint64_t t1 = ctz64(~spmpaddr);
        uint64_t range = (1ULL << (t1+3));

        // spmpaddr reg encodes bits 55-2 of a
        // 56 bit physical address for RV64
        uint64_t base = mbits(spmpaddr, 63, t1) << 2;
        AddrRange this_range(base, base+range);
        return this_range;
    }
}

} // namespace gem5
