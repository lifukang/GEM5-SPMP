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

#ifndef __ARCH_RISCV_SPMP_HH__
#define __ARCH_RISCV_SPMP_HH__

#include "arch/generic/tlb.hh"
#include "arch/riscv/isa.hh"
#include "base/addr_range.hh"
#include "base/types.hh"
#include "mem/packet.hh"
#include "params/SPMP.hh"
#include "sim/sim_object.hh"

/**
 * @file
 * SPMP header file.
 */

namespace gem5
{


/**
 * This class helps to implement RISCV's physical memory
 * protection (spmp) primitive.
 * @todo Add statistics and debug prints.
 */
class SPMP : public SimObject
{
  public:
    PARAMS(SPMP);
    SPMP(const Params &params);

  private:
    /** spmp表中entry的最大数量 */
    int spmpEntries;

    /** spmp不同寻址方式枚举*/
    enum spmpAmatch
    {
        SPMP_OFF,
        SPMP_TOR,
        SPMP_NA4,
        SPMP_NAPOT
    };

    /** cfg寄存器的读权限位的mask*/
    const uint8_t SPMP_READ = 1 << 0;

    /** cfg寄存器的写权限位的mask*/
    const uint8_t SPMP_WRITE = 1 << 1;

    /** cfg寄存器的执行权限位的mask*/
    const uint8_t SPMP_EXEC = 1 << 2;

    /** cfg寄存器的地址模式权限位的mask*/
    const uint8_t SPMP_A_MASK = 3 << 3;

    /** cfg寄存器的S-mode only权限位的mask*/
    const uint8_t SPMP_S = 1 << 7;//S=1 表示S-mode only S=0 表示U-mode only

    /** 活跃的spmp entry数量 */
    int numRules;//


    /** spmp entry格式*/
    struct SPmpEntry
    {
        /** 每个spmp entry对应的地址区间 */
        AddrRange spmpAddr = AddrRange(0, 0);
        /** spmp entry存储单条地址 */
        Addr rawAddr;//
        /** cfg寄存器的值 */
        uint8_t spmpCfg = 0;
    };

    /** 存储spmp entry的结构 */
    std::vector<SPmpEntry> spmpTable;

  public:
    /**
     * spmpCheck checks if a particular memory access
     * is allowed based on the spmp rules.
     * @param req memory request.
     * @param mode mode of request (read, write, execute).
     * @param pmode current privilege mode of memory (U, S, M).
     * @param tc thread context.
     * @param vaddr optional parameter to pass vaddr of original
     * request for which a page table walk is consulted by spmp unit
     * @return Fault.
     */
    Fault spmpCheck( RiscvISA::STATUS status,const RequestPtr &req, BaseMMU::Mode mode,
                  RiscvISA::PrivilegeMode pmode, ThreadContext *tc,
                  Addr vaddr = 0);

    /**
     * spmpUpdateCfg updates the spmpcfg for a spmp
     * entry and calls spmpUpdateRule to update the
     * rule of corresponding spmp entry.
     * @param spmp_index spmp entry index.
     * @param this_cfg value to be written to spmpcfg.
     * @returns true if update spmpicfg success
     */
    bool spmpUpdateCfg(uint32_t spmp_index, uint8_t this_cfg);

    /**
     * spmpUpdateAddr updates the spmpaddr for a spmp
     * entry and calls spmpUpdateRule to update the
     * rule of corresponding spmp entry.
     * @param spmp_index spmp entry index.
     * @param this_addr value to be written to spmpaddr.
     * @returns true if update spmpaddri success
     */
    bool spmpUpdateAddr(uint32_t spmp_index, Addr this_addr);

    /**
     * spmpReset reset when reset signal in trigger from
     * CPU.
     */
    void spmpReset();

  private:
    /**
     * This function is called during a memory
     * access to determine if the spmp table
     * should be consulted for this access.
     * @param spmode current privilege mode of memory (U, S, M).
     * @param tc thread context.
     * @return true or false.
     */
    bool shouldCheckSPMP(RiscvISA::PrivilegeMode pmode, ThreadContext *tc);

    /**
     * createAddrfault creates an address fault
     * if the spmp checks fail to pass for a given
     * access. This function is used by spmpCheck().
     * given spmp entry depending on the value
     * of spmpaddr and spmpcfg for that entry.
     * @param vaddr virtual address of the access.
     * @param mode mode of access(read, write, execute).
     * @return Fault.
     */
    Fault createAddrfault(Addr vaddr, BaseMMU::Mode mode);

    /**
     * spmpUpdateRule updates the spmp rule for a
     * given spmp entry depending on the value
     * of spmpaddr and spmpcfg for that entry.
     * @param spmp_index spmp entry index.
     */
    void spmpUpdateRule(uint32_t spmp_index);

    /**
     * spmpGetAField extracts the A field (address matching mode)
     * from an input spmpcfg register
     * @param cfg spmpcfg register value.
     * @return The A field.
     */
    inline uint8_t spmpGetAField(uint8_t cfg);

    /**
     * This function decodes a spmpaddr register value
     * into an address range when A field of spmpcfg
     * register is set to NAPOT mode (naturally aligned
     * power of two region).
     * @param spmpaddr input address from a spmp entry.
     * @return an address range.
     */
    inline AddrRange spmpDecodeNapot(Addr spmpaddr);

};


} // namespace gem5

#endif // __ARCH_RISCV_SPMP_HH__
