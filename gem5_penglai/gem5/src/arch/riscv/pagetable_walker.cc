/*
 * Copyright (c) 2012 ARM Limited
 * Copyright (c) 2020 Barkhausen Institut
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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

#include "arch/riscv/pagetable_walker.hh"

#include <memory>

#include "arch/riscv/faults.hh"
#include "arch/riscv/page_size.hh"
#include "arch/riscv/pagetable.hh"
#include "arch/riscv/tlb.hh"
#include "base/bitfield.hh"
#include "base/trie.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/PageTableWalker.hh"
#include "debug/RiscvMisc.hh"
#include "mem/packet_access.hh"
#include "mem/request.hh"
#include "debug/SPMP.hh"
#include "debug/PMP.hh"

namespace gem5
{

namespace RiscvISA {

Fault
Walker::start(ThreadContext * _tc, BaseMMU::Translation *_translation,
              const RequestPtr &_req, BaseMMU::Mode _mode)
{
    // TODO: in timing mode, instead of blocking when there are other
    // outstanding requests, see if this request can be coalesced with
    // another one (i.e. either coalesce or start walk)
    WalkerState * newState = new WalkerState(this, _translation, _req);
    //this指调用该函数的对象，此处为Walker对象 new的walkerstate指向当前walker对象this  walkerstate对象赋值req  translation等等
    newState->initState(_tc, _mode, sys->isTimingMode());//读寄存器 timing start tc赋值
    if (currStates.size()) {//若currstate list有请求且是timing请求则将新创建的walkerstate压入队列 currstate 将walkerstate压入currstate队列
        assert(newState->isTiming());//assert(0)中断  1继续
        DPRINTF(PageTableWalker, "Walks in progress: %d\n", currStates.size());
        currStates.push_back(newState);
        return NoFault;
    } else {//若currstate list无请求 直接压入 开始walk。如果不是timing
        currStates.push_back(newState);
        Fault fault = newState->startWalk();
        if (!newState->isTiming()) {
            currStates.pop_front();
            delete newState;
        }
        return fault;
    }
}

Fault
Walker::startFunctional(ThreadContext * _tc, Addr &addr, unsigned &logBytes,
              BaseMMU::Mode _mode)
{
    funcState.initState(_tc, _mode);
    return funcState.startFunctional(addr, logBytes);
}

bool
Walker::WalkerPort::recvTimingResp(PacketPtr pkt)
{
    return walker->recvTimingResp(pkt);
}

bool
Walker::recvTimingResp(PacketPtr pkt)
{
    WalkerSenderState * senderState =
        dynamic_cast<WalkerSenderState *>(pkt->popSenderState());
    WalkerState * senderWalk = senderState->senderWalk;
    bool walkComplete = senderWalk->recvPacket(pkt);
    delete senderState;
    if (walkComplete) {
        std::list<WalkerState *>::iterator iter;
        for (iter = currStates.begin(); iter != currStates.end(); iter++) {
            WalkerState * walkerState = *(iter);
            if (walkerState == senderWalk) {
                iter = currStates.erase(iter);
                break;
            }
        }
        delete senderWalk;
        // Since we block requests when another is outstanding, we
        // need to check if there is a waiting request to be serviced
        if (currStates.size() && !startWalkWrapperEvent.scheduled())
            // delay sending any new requests until we are finished
            // with the responses
            schedule(startWalkWrapperEvent, clockEdge());
    }
    return true;
}

void
Walker::WalkerPort::recvReqRetry()
{
    walker->recvReqRetry();
}

void
Walker::recvReqRetry()
{
    std::list<WalkerState *>::iterator iter;
    for (iter = currStates.begin(); iter != currStates.end(); iter++) {
        WalkerState * walkerState = *(iter);
        if (walkerState->isRetrying()) {
            walkerState->retry();
        }
    }
}

bool Walker::sendTiming(WalkerState* sendingState, PacketPtr pkt)
{
    WalkerSenderState* walker_state = new WalkerSenderState(sendingState);
    pkt->pushSenderState(walker_state);
    if (port.sendTimingReq(pkt)) {
        return true;
    } else {
        // undo the adding of the sender state and delete it, as we
        // will do it again the next time we attempt to send it
        pkt->popSenderState();
        delete walker_state;
        return false;
    }

}

Port &
Walker::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "port")
        return port;
    else
        return ClockedObject::getPort(if_name, idx);
}

void
Walker::WalkerState::initState(ThreadContext * _tc,
        BaseMMU::Mode _mode, bool _isTiming)
{
    assert(state == Ready);
    started = false;
    tc = _tc;
    mode = _mode;
    timing = _isTiming;
    // fetch these now in case they change during the walk
    status = tc->readMiscReg(MISCREG_STATUS);
    pmode = walker->tlb->getMemPriv(tc, mode);
    satp = tc->readMiscReg(MISCREG_SATP);
    assert(satp.mode == AddrXlateMode::SV39);
}

void
Walker::startWalkWrapper()
{
    unsigned num_squashed = 0;
    WalkerState *currState = currStates.front();
    while ((num_squashed < numSquashable) && currState &&
        currState->translation->squashed()) {
        currStates.pop_front();
        num_squashed++;

        DPRINTF(PageTableWalker, "Squashing table walk for address %#x\n",
            currState->req->getVaddr());

        // finish the translation which will delete the translation object
        currState->translation->finish(
            std::make_shared<UnimpFault>("Squashed Inst"),
            currState->req, currState->tc, currState->mode);

        // delete the current request if there are no inflight packets.
        // if there is something in flight, delete when the packets are
        // received and inflight is zero.
        if (currState->numInflight() == 0) {
            delete currState;
        } else {
            currState->squash();
        }

        // check the next translation request, if it exists
        if (currStates.size())
            currState = currStates.front();
        else
            currState = NULL;
    }
    if (currState && !currState->wasStarted())
        currState->startWalk();
}

Fault
Walker::WalkerState::startWalk()
{
    Fault fault = NoFault;
    assert(!started);
    started = true;
    setupWalk(req->getVaddr());//得到req的虚拟地址
    if (timing) {
        nextState = state;
        state = Waiting;
        timingFault = NoFault;
        sendPackets();
    } else {
        do {
            walker->port.sendAtomic(read);//得到延迟
            PacketPtr write = NULL;
            fault = stepWalk(write);
            assert(fault == NoFault || read == NULL);
            state = nextState;
            nextState = Ready;
            if (write)
                walker->port.sendAtomic(write);
        } while (read);//读写都用port.sendAtomic 循环read
        state = Ready;
        nextState = Waiting;
    }
    return fault;
}

Fault
Walker::WalkerState::startFunctional(Addr &addr, unsigned &logBytes)
{
    Fault fault = NoFault;
    assert(!started);
    started = true;
    setupWalk(addr);

    do {
        walker->port.sendFunctional(read);
        // On a functional access (page table lookup), writes should
        // not happen so this pointer is ignored after stepWalk
        PacketPtr write = NULL;
        fault = stepWalk(write);
        assert(fault == NoFault || read == NULL);
        state = nextState;
        nextState = Ready;
    } while (read);
    logBytes = entry.logBytes;
    addr = entry.paddr << PageShift;

    return fault;
}

Fault
Walker::WalkerState::stepWalk(PacketPtr &write)
{
    assert(state != Ready && state != Waiting);
    Fault fault = NoFault;
    Fault fault1 = NoFault;
    Fault fault2 = NoFault;
    write = NULL;
    PTESv39 pte = read->getLE<uint64_t>();
    Addr nextRead = 0;
    bool doWrite = false;
    bool doTLBInsert = false;
    bool doEndWalk = false;

    
    DPRINTF(PageTableWalker, "Got level%d PTE: %#x\n", level, pte);
    
    

    fault1 = walker->spmp->spmpCheck(status,read->req, BaseMMU::Read,RiscvISA::PrivilegeMode::PRV_S, tc, entry.vaddr);
    // step 2:
    // Performing PMA/PMP checks on physical address of PTE
     if (read->req->hasVaddr()) {
        DPRINTF(SPMP, "PageTable Got level%d PTE: %#x： Checking spmp permissions for va:  %#x,pa: %#x\n",level, pte,read->req->getVaddr(),read->req->getPaddr());
    }
    else {
        DPRINTF(SPMP, "PageTable Got level%d PTE: %#x： Checking spmp permissions for pa: %#x\n",level, pte,read->req->getPaddr());
    }
    if(fault1== NoFault){
       DPRINTF(SPMP, "Checking spmp permissions SUCCESS!\n");
    }
    else
        DPRINTF(SPMP, "Checking spmp permissions FAILED!\n");

    walker->pma->check(read->req);//对页表的物理地址进行PMA-CHECK&&进行PMP-CHECK
    // Effective privilege mode for pmp checks for page table
    // walks is S mode according to specs
    fault2 = walker->pmp->pmpCheck(read->req, BaseMMU::Read,
                    RiscvISA::PrivilegeMode::PRV_S, tc, entry.vaddr);
    
    if((fault1 == NoFault)&&(fault2 == NoFault))
            fault=NoFault;

    if (fault== NoFault) {//PMP-CHECK和SPMP-CHECK通过之后进行位权限检查
        // step 3:
        if (!pte.v || (!pte.r && pte.w)) {//权限检查失败
            doEndWalk = true;
            DPRINTF(PageTableWalker, "PTE invalid, raising PF\n");
            fault = pageFault(pte.v);
        }
        else {//权限检查通过
            // step 4:
            if (pte.r || pte.x) {//最后一层页表，叶子节点
                // step 5: leaf PTE
                doEndWalk = true;
                fault = walker->tlb->checkPermissions(status, pmode,
                                                    entry.vaddr, mode, pte);

                // step 6
                if (fault == NoFault) {
                    if (level >= 1 && pte.ppn0 != 0) {
                        DPRINTF(PageTableWalker,
                                "PTE has misaligned PPN, raising PF\n");
                        fault = pageFault(true);
                    }
                    else if (level == 2 && pte.ppn1 != 0) {
                        DPRINTF(PageTableWalker,
                                "PTE has misaligned PPN, raising PF\n");
                        fault = pageFault(true);
                    }
                }

                if (fault == NoFault) {
                    // step 7
                    if (!pte.a) {
                        pte.a = 1;
                        doWrite = true;
                    }
                    if (!pte.d && mode == BaseMMU::Write) {
                        pte.d = 1;
                        doWrite = true;
                    }
                    // Performing PMA/PMP checks

                    if (doWrite) {

                        // this read will eventually become write
                        // if doWrite is True
                        fault1 = walker->spmp->spmpCheck(status,read->req, BaseMMU::Write,RiscvISA::PrivilegeMode::PRV_S, tc, entry.vaddr);

                        if (read->req->hasVaddr()) {
                            DPRINTF(SPMP, "PageTable write Got level%d PTE: %#x： Checking spmp permissions for va:  %#x,pa: %#x\n",level, pte,read->req->getVaddr(),read->req->getPaddr());
                        }
                        else {
                            DPRINTF(SPMP, "PageTable write Got level%d PTE: %#x： Checking spmp permissions for pa: %#x\n",level, pte,read->req->getPaddr());
                        }
                        if(fault1== NoFault){
                            DPRINTF(SPMP, "Checking spmp permissions SUCCESS!\n");
                        }
                        else
                            DPRINTF(SPMP, "Checking spmp permissions FAILED!\n");

                        walker->pma->check(read->req);

                        fault2 = walker->pmp->pmpCheck(read->req,//write 模式
                                            BaseMMU::Write, pmode, tc, entry.vaddr);
                        
                        if((fault1 == NoFault)&&(fault2 == NoFault))
                            fault=NoFault;

                    }
                    // perform step 8 only if pmp checks pass
                    if (fault == NoFault) {

                        // step 8
                        entry.logBytes = PageShift + (level * LEVEL_BITS);
                        entry.paddr = pte.ppn;
                        entry.vaddr &= ~((1 << entry.logBytes) - 1);
                        entry.pte = pte;
                        // put it non-writable into the TLB to detect
                        // writes and redo the page table walk in order
                        // to update the dirty flag.
                        if (!pte.d && mode != BaseMMU::Write)
                            entry.pte.w = 0;
                        doTLBInsert = true;
                    }
                }
            } else {//一层一层查找页表
                level--;
                if (level < 0) {
                    DPRINTF(PageTableWalker, "No leaf PTE found,"
                                                  "raising PF\n");
                    doEndWalk = true;
                    fault = pageFault(true);
                } else {//循环赋值
                    Addr shift = (PageShift + LEVEL_BITS * level);
                    Addr idx = (entry.vaddr >> shift) & LEVEL_MASK;
                    nextRead = (pte.ppn << PageShift) + (idx * sizeof(pte));
                    nextState = Translate;
                }
            }
        }
    } else {
        doEndWalk = true;//没有通过权限检查
    }
    PacketPtr oldRead = read;
    Request::Flags flags = oldRead->req->getFlags();

    if (doEndWalk) {
        // If we need to write, adjust the read packet to write the modified
        // value back to memory.
        if (!functional && doWrite) {//写的时候，将read request转换成write request
            DPRINTF(PageTableWalker, "Writing level%d PTE to %#x: %#x\n",
                level, oldRead->getAddr(), pte);
            write = oldRead;
            write->setLE<uint64_t>(pte);
            write->cmd = MemCmd::WriteReq;
            read = NULL;
        } else {
            write = NULL;
        }

        if (doTLBInsert) {
            if (!functional)
                walker->tlb->insert(entry.vaddr, entry);
            else {
                DPRINTF(PageTableWalker, "Translated %#x -> %#x\n",
                        entry.vaddr, entry.paddr << PageShift |
                        (entry.vaddr & mask(entry.logBytes)));
            }
        }
        endWalk();
    }
    else {
        //If we didn't return, we're setting up another read.
        RequestPtr request = std::make_shared<Request>(//建立下一个请求
            nextRead, oldRead->getSize(), flags, walker->requestorId);

        delete oldRead;
        oldRead = nullptr;

        read = new Packet(request, MemCmd::ReadReq);
        read->allocate();

        DPRINTF(PageTableWalker,
                "Loading level%d PTE from %#x\n", level, nextRead);
    }

    return fault;
}

void
Walker::WalkerState::endWalk()
{
    nextState = Ready;
    delete read;
    read = NULL;
}

void
Walker::WalkerState::setupWalk(Addr vaddr)
{
    vaddr = Addr(sext<VADDR_BITS>(vaddr));

    Addr shift = PageShift + LEVEL_BITS * 2;//偏移=偏移量+页表级别*2（3级页表 2*9=18）
    Addr idx = (vaddr >> shift) & LEVEL_MASK;//页表的vaddr右移shift位找到页目录表的索引
    Addr topAddr = (satp.ppn << PageShift) + (idx * sizeof(PTESv39));//找到虚拟地址页目录表对应idx的地址 PPN域保存根页表的物理地址
    level = 2;

    DPRINTF(PageTableWalker, "Performing table walk for address %#x\n", vaddr);
    DPRINTF(PageTableWalker, "Loading level%d PTE from %#x\n", level, topAddr);

    state = Translate;
    nextState = Ready;
    entry.vaddr = vaddr;
    entry.asid = satp.asid;

    Request::Flags flags = Request::PHYSICAL;
    RequestPtr request = std::make_shared<Request>(
        topAddr, sizeof(PTESv39), flags, walker->requestorId);

    read = new Packet(request, MemCmd::ReadReq);
    read->allocate();
}

bool
Walker::WalkerState::recvPacket(PacketPtr pkt)
{
    assert(pkt->isResponse());
    assert(inflight);
    assert(state == Waiting);
    inflight--;
    if (squashed) {
        // if were were squashed, return true once inflight is zero and
        // this WalkerState will be freed there.
        return (inflight == 0);
    }
    if (pkt->isRead()) {
        // should not have a pending read it we also had one outstanding
        assert(!read);

        // @todo someone should pay for this
        pkt->headerDelay = pkt->payloadDelay = 0;

        state = nextState;
        nextState = Ready;
        PacketPtr write = NULL;
        read = pkt;
        timingFault = stepWalk(write);
        state = Waiting;
        assert(timingFault == NoFault || read == NULL);
        if (write) {
            writes.push_back(write);
        }
        sendPackets();
    } else {
        delete pkt;

        sendPackets();
    }
    if (inflight == 0 && read == NULL && writes.size() == 0) {
        state = Ready;
        nextState = Waiting;
        if (timingFault == NoFault) {
            /*
             * Finish the translation. Now that we know the right entry is
             * in the TLB, this should work with no memory accesses.
             * There could be new faults unrelated to the table walk like
             * permissions violations, so we'll need the return value as
             * well.
             */
            Addr vaddr = req->getVaddr();
            vaddr = Addr(sext<VADDR_BITS>(vaddr));
            Addr paddr = walker->tlb->translateWithTLB(vaddr, satp.asid, mode);
            req->setPaddr(paddr);
            walker->pma->check(req);

            // do pmp check if any checking condition is met.
            // timingFault will be NoFault if pmp checks are
            // passed, otherwise an address fault will be returned.
            timingFault = walker->pmp->pmpCheck(req, mode, pmode, tc);

            // Let the CPU continue.
            translation->finish(timingFault, req, tc, mode);
        } else {
            // There was a fault during the walk. Let the CPU know.
            translation->finish(timingFault, req, tc, mode);
        }
        return true;
    }

    return false;
}

void
Walker::WalkerState::sendPackets()
{
    //If we're already waiting for the port to become available, just return.
    if (retrying)
        return;

    //Reads always have priority
    if (read) {
        PacketPtr pkt = read;
        read = NULL;
        inflight++;
        if (!walker->sendTiming(this, pkt)) {
            retrying = true;
            read = pkt;
            inflight--;
            return;
        }
    }
    //Send off as many of the writes as we can.
    while (writes.size()) {
        PacketPtr write = writes.back();
        writes.pop_back();
        inflight++;
        if (!walker->sendTiming(this, write)) {
            retrying = true;
            writes.push_back(write);
            inflight--;
            return;
        }
    }
}

unsigned
Walker::WalkerState::numInflight() const
{
    return inflight;
}

bool
Walker::WalkerState::isRetrying()
{
    return retrying;
}

bool
Walker::WalkerState::isTiming()
{
    return timing;
}

bool
Walker::WalkerState::wasStarted()
{
    return started;
}

void
Walker::WalkerState::squash()
{
    squashed = true;
}

void
Walker::WalkerState::retry()
{
    retrying = false;
    sendPackets();
}

Fault
Walker::WalkerState::pageFault(bool present)
{
    DPRINTF(PageTableWalker, "Raising page fault.\n");
    return walker->tlb->createPagefault(entry.vaddr, mode);
}

} // namespace RiscvISA
} // namespace gem5
