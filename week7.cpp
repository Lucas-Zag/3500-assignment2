System::Console::ReadKey(true);


#pragma once
#include <NetworkedModule.h>
#include <SMObjects.h>

using namespace System;
using namespace System::Threading;

ref class LiDAR : public NetworkedModule
{
public:
    // 通过构造函数把 SM 指针交进来（C++/CLI，方便设置到基类的 SM_TM_）
    LiDAR(SM_ThreadManagement^ sm_tm, SM_Lidar^ sm_lidar) {
        SM_TM_ = sm_tm;
        SM_L_  = sm_lidar;
    }

    // Week8 不要求真实网络通信，下面两个是占位即可
    virtual error_state connect(String^ hostName, int portNumber) override { return error_state::SUCCESS; }
    virtual error_state communicate() override { return error_state::SUCCESS; }

    virtual error_state processSharedMemory() override { return error_state::SUCCESS; }

    virtual bool getShutdownFlag() override {
        return (SM_TM_ != nullptr) && (SM_TM_->shutdown != 0);
    }

    virtual void threadFunction() override;

private:
    SM_Lidar^ SM_L_;
    void writeScanToSharedMemory(const array<double>^ x, const array<double>^ y);
};



#include "LiDAR.h"
#include <cmath>

using namespace System;
using namespace System::Threading;
using namespace System::Net::Sockets;
using namespace System::Text;
using namespace System::Globalization;

// ===== ctor =====
LiDAR::LiDAR(SM_ThreadManagement^ sm_tm, SM_Lidar^ sm_l) {
    SM_TM_ = sm_tm;
    SM_L_  = sm_l;
}

// ===== SM writer =====
void LiDAR::writeScanToSharedMemory(const array<double>^ x, const array<double>^ y)
{
    if (!SM_L_) return;
    Monitor::Enter(SM_L_->lockObject);
    try {
        for (int i = 0; i < x->Length && i < SM_L_->x->Length; ++i) {
            SM_L_->x[i] = x[i];
            SM_L_->y[i] = y[i];
        }
    }
    finally { Monitor::Exit(SM_L_->lockObject); }
}

// ===== main thread loop =====
void LiDAR::threadFunction()
{
    Console::WriteLine("[LiDAR] Connecting to simulator 127.0.0.1:23000 ...");
    TcpClient^ client = gcnew TcpClient();
    try { client->Connect("127.0.0.1", 23000); }
    catch (Exception^ e) {
        Console::WriteLine("[LiDAR] Connect failed: {0}", e->Message);
        return;
    }
    NetworkStream^ stream = client->GetStream();

    // --- 1) 认证：zID(不带 'z') + '\n' → 期待 "OK\n"
    // TODO: 改成你的学号：例如 "5628323\n"
    String^ zid = "1234567\n";
    array<Byte>^ auth = Encoding::ASCII->GetBytes(zid);
    stream->Write(auth, 0, auth->Length);

    array<Byte>^ ackBuf = gcnew array<Byte>(64);
    int ackLen = stream->Read(ackBuf, 0, ackBuf->Length);
    String^ ack = (ackLen > 0) ? Encoding::ASCII->GetString(ackBuf, 0, ackLen) : "";
    if (!ack->Contains("OK")) {
        Console::WriteLine("[LiDAR] Auth failed, resp='{0}'", ack);
        return;
    }
    Console::WriteLine("[LiDAR] Authenticated with simulator.");

    // --- 2) 请求帧：STX(0x02) + "sRN LMDscandata" + ETX(0x03)
    array<Byte>^ cmd = Encoding::ASCII->GetBytes("sRN LMDscandata");
    array<Byte>^ req = gcnew array<Byte>(cmd->Length + 2);
    req[0] = 0x02;                                 // STX
    Array::Copy(cmd, 0, req, 1, cmd->Length);
    req[req->Length - 1] = 0x03;                   // ETX

    // --- 3) 接收缓冲
    array<Byte>^ rx = gcnew array<Byte>(16384);
    String^ carry = "";                            // 处理半包/粘包

    const int N = STANDARD_LIDAR_LENGTH;           // 361
    array<double>^ x = gcnew array<double>(N);
    array<double>^ y = gcnew array<double>(N);
    int frameId = 0;

    while (!getShutdownFlag()) {
        // 发送请求
        stream->Write(req, 0, req->Length);

        // 读取到 ETX；去除 STX/ETX，留下纯 ASCII 帧
        String^ frame = nullptr;
        for (;;) {
            int m = 0;
            try { m = stream->Read(rx, 0, rx->Length); }
            catch (Exception^ e) { Console::WriteLine("[LiDAR] Read error: {0}", e->Message); return; }
            if (m <= 0) { Console::WriteLine("[LiDAR] connection closed."); return; }

            String^ chunk = Encoding::ASCII->GetString(rx, 0, m);
            carry += chunk;

            int stx = carry->IndexOf((wchar_t)0x02);
            int etx = carry->IndexOf((wchar_t)0x03);
            if (stx >= 0 && etx > stx) {
                frame = carry->Substring(stx + 1, etx - stx - 1);
                carry = carry->Substring(etx + 1);
                break;
            }
            if (carry->Length > 60000) { Console::WriteLine("[LiDAR] carry too long, reset."); carry = ""; }
        }

        // === 4) 解析：稳健定位 DIST1 的“点数”和数据起始下标 ===
        array<String^>^ tok = frame->Split(' ', StringSplitOptions::RemoveEmptyEntries);
        int idx = Array::IndexOf(tok, "DIST1");
        if (idx < 0) {
            int preview = Math::Min(frame->Length, 120);
            Console::WriteLine("[LiDAR] DIST1 not found. head='{0}'", frame->Substring(0, preview));
            continue;
        }

        // 有些实现会在 DIST1 后给出 scale/offset 等，点数并非紧随其后
        int count = -1;
        int dataStart = -1;
        for (int j = idx + 1; j < Math::Min(idx + 12, tok->Length); ++j) {
            int tryCount = 0;
            bool ok = Int32::TryParse(tok[j], NumberStyles::HexNumber, nullptr, tryCount);
            // 合理范围过滤（361 属于此范围），并确保后续 token 足够承载
            if (ok && tryCount >= 10 && tryCount <= 2000) {
                if (j + 1 + tryCount <= tok->Length) {
                    count = tryCount;
                    dataStart = j + 1;
                    break;
                }
            }
        }

        if (count != N || dataStart < 0) {
            Console::WriteLine("[LiDAR] count/offset unresolved. got count={0}, tokens={1}", count, tok->Length);
            continue;
        }

        // === 5) 极坐标(mm) → 笛卡尔(m)；0..180°，步距 0.5°
        double minr = 1e9, maxr = -1e9;
        for (int i = 0; i < N; ++i) {
            int r_mm = 0;
            Int32::TryParse(tok[dataStart + i], NumberStyles::HexNumber, nullptr, r_mm);
            double r   = r_mm / 1000.0;                 // mm → m
            double deg = 0.5 * i;                       // 0..180
            double rad = deg * Math::PI / 180.0;
            x[i] = r * Math::Cos(rad);
            y[i] = r * Math::Sin(rad);
            if (r > 0) { if (r < minr) minr = r; if (r > maxr) maxr = r; }
        }

        // === 6) 写共享内存 + 打印“live”证据 ===
        writeScanToSharedMemory(x, y);

        Console::Write("[LiDAR] frame {0}  n={1}  r[min,max]=[{2:F2},{3:F2}]  first10: ",
            ++frameId, N, (minr<1e8?minr:0), (maxr>-1e8?maxr:0));
        for (int i = 0; i < 10; ++i) Console::Write("( {0:F3},{1:F3} ) ", x[i], y[i]);
        Console::WriteLine();

        // 心跳（可选）
        if (SM_TM_) {
            Monitor::Enter(SM_TM_->lockObject);
            try { SM_TM_->heartbeat |= bit_LIDAR; }
            finally { Monitor::Exit(SM_TM_->lockObject); }
        }

        Thread::Sleep(40); // ~25 Hz
    }

    Console::WriteLine("[LiDAR] thread exit.");
}






#pragma once
#include <UGVModule.h>
#include <SMObjects.h>

ref class LiDAR;            // 前置声明（与各模块解耦）
ref class Display;
ref class GNSS;
ref class Controller;
ref class VC;
ref class CrashAvoidance;

ref class ThreadManagement : public UGVModule {
public:
    // Create shared memory objects
    error_state setupSharedMemory();

    // Send/Receive data from shared memory structures
    error_state processSharedMemory() override;

    // Shutdown all modules in the software
    void shutdownModules();

    // Get Shutdown signal for module, from Thread Management SM
    bool getShutdownFlag() override;

    // Thread function for TMM
    void threadFunction() override;

private:
    // 共享内存
    SM_Lidar^    SM_L_   = nullptr;
    SM_GNSS^     SM_G_   = nullptr;
    SM_VehicleControl^ SM_VC_ = nullptr;

    // 其他模块实例
    LiDAR^          lidar_ = nullptr;
    Display^        display_ = nullptr;
    GNSS^           gnss_ = nullptr;
    Controller^     controller_ = nullptr;
    VC^             vc_ = nullptr;
    CrashAvoidance^ crash_ = nullptr;
};



#include "TMM.h"
#include "LiDAR.h"       // 只需要 LiDAR 的定义；其他模块放各自头文件（见下）
#include "Display.h"
#include "GNSS.h"
#include "Controller.h"
#include "VC.h"
#include "CrashAvoidance.h"

using namespace System;
using namespace System::Threading;

error_state ThreadManagement::setupSharedMemory() {
    // 创建并挂到基类指针（供所有模块共享）
    SM_TM_ = gcnew SM_ThreadManagement();
    SM_L_  = gcnew SM_Lidar();
    SM_G_  = gcnew SM_GNSS();
    SM_VC_ = gcnew SM_VehicleControl();

    // 心跳 WatchList 可选；Week8 不强制用，演示时可忽略
    return error_state::SUCCESS;
}

error_state ThreadManagement::processSharedMemory() {
    // 这里可以做一点点心跳监测（演示“通信”），例如每个周期把 heartbeat 清零
    if (SM_TM_) {
        Monitor::Enter(SM_TM_->lockObject);
        try { SM_TM_->heartbeat = 0; }
        finally { Monitor::Exit(SM_TM_->lockObject); }
    }
    return error_state::SUCCESS;
}

void ThreadManagement::shutdownModules() {
    if (SM_TM_) {
        SM_TM_->shutdown = 0xFF; // 非 0 即触发所有模块退出
    }
}

bool ThreadManagement::getShutdownFlag() {
    // TMM 自己也根据 SM 的关机标志退出（保持风格一致）
    return (SM_TM_ != nullptr) && (SM_TM_->shutdown != 0);
}

void ThreadManagement::threadFunction() {
    Console::WriteLine("[TMM] starting…");
    setupSharedMemory();

    // —— 创建各模块并传入共享内存 —— //
    lidar_      = gcnew LiDAR(SM_TM_, SM_L_);
    display_    = gcnew Display(SM_TM_, SM_L_);         // 下文提供最小骨架
    gnss_       = gcnew GNSS(SM_TM_, SM_G_);
    controller_ = gcnew Controller(SM_TM_, SM_L_, SM_G_, SM_VC_);
    vc_         = gcnew VC(SM_TM_, SM_VC_);
    crash_      = gcnew CrashAvoidance(SM_TM_, SM_L_, SM_VC_);

    // —— 启动线程 —— //
    Thread^ thL = gcnew Thread(gcnew ThreadStart(lidar_,      &LiDAR::threadFunction));
    Thread^ thD = gcnew Thread(gcnew ThreadStart(display_,    &Display::threadFunction));
    Thread^ thG = gcnew Thread(gcnew ThreadStart(gnss_,       &GNSS::threadFunction));
    Thread^ thC = gcnew Thread(gcnew ThreadStart(controller_, &Controller::threadFunction));
    Thread^ thV = gcnew Thread(gcnew ThreadStart(vc_,         &VC::threadFunction));
    Thread^ thA = gcnew Thread(gcnew ThreadStart(crash_,      &CrashAvoidance::threadFunction));

    thL->Start(); thD->Start(); thG->Start(); thC->Start(); thV->Start(); thA->Start();

    Console::WriteLine("[TMM] Press 'q' to shutdown.");

    // —— 键盘监听（C++/CLI，用 Console::KeyAvailable） —— //
    while (!getShutdownFlag()) {
        if (Console::KeyAvailable) {
            auto key = Console::ReadKey(true).Key;
            if (key == ConsoleKey::Q) {
                Console::WriteLine("[TMM] Shutdown requested.");
                shutdownModules();
                break;
            }
        }
        // 演示通信：周期性清 heartbeat
        processSharedMemory();
        Thread::Sleep(100);
    }

    // —— 等待所有线程退出 —— //
    thL->Join(); thD->Join(); thG->Join(); thC->Join(); thV->Join(); thA->Join();
    Console::WriteLine("[TMM] all threads exited.");
}




#include "TMM.h"
using namespace System;
using namespace System::Threading;

int main(array<System::String ^> ^)
{
    ThreadManagement^ tmm = gcnew ThreadManagement();
    Thread^ thTM = gcnew Thread(gcnew ThreadStart(tmm, &ThreadManagement::threadFunction));
    thTM->Start();
    thTM->Join();
    return 0;
}





// Display.h
#pragma once
#include <UGVModule.h>
#include <SMObjects.h>
using namespace System;
using namespace System::Threading;

ref class Display : public UGVModule {
public:
    Display(SM_ThreadManagement^ sm_tm, SM_Lidar^ sm_l) { SM_TM_ = sm_tm; SM_L_ = sm_l; }
    virtual error_state processSharedMemory() override { return error_state::SUCCESS; }
    virtual bool getShutdownFlag() override { return (SM_TM_!=nullptr) && (SM_TM_->shutdown!=0); }
    virtual void threadFunction() override;
private:
    SM_Lidar^ SM_L_;
};

// Display.cpp
#include "Display.h"
void Display::threadFunction() {
    while (!getShutdownFlag()) {
        // 心跳
        if (SM_TM_) { Monitor::Enter(SM_TM_->lockObject); try { SM_TM_->heartbeat |= bit_DISPLAY; } finally { Monitor::Exit(SM_TM_->lockObject); } }
        Thread::Sleep(200);
    }
    Console::WriteLine("[Display] thread exit.");
}




// GNSS.h
#pragma once
#include <UGVModule.h>
#include <SMObjects.h>
using namespace System;
using namespace System::Threading;

ref class GNSS : public UGVModule {
public:
    GNSS(SM_ThreadManagement^ sm_tm, SM_GNSS^ sm_g) { SM_TM_ = sm_tm; SM_G_ = sm_g; }
    virtual error_state processSharedMemory() override { return error_state::SUCCESS; }
    virtual bool getShutdownFlag() override { return (SM_TM_!=nullptr) && (SM_TM_->shutdown!=0); }
    virtual void threadFunction() override {
        while (!getShutdownFlag()) {
            if (SM_TM_) { Monitor::Enter(SM_TM_->lockObject); try { SM_TM_->heartbeat |= bit_GNSS; } finally { Monitor::Exit(SM_TM_->lockObject); } }
            Thread::Sleep(150);
        }
        System::Console::WriteLine("[GNSS] thread exit.");
    }
private: SM_GNSS^ SM_G_;
};


// Controller.h
#pragma once
#include <UGVModule.h>
#include <SMObjects.h>
using namespace System;
using namespace System::Threading;

ref class Controller : public UGVModule {
public:
    Controller(SM_ThreadManagement^ sm_tm, SM_Lidar^ sm_l, SM_GNSS^ sm_g, SM_VehicleControl^ sm_vc) {
        SM_TM_ = sm_tm; SM_L_ = sm_l; SM_G_ = sm_g; SM_VC_ = sm_vc;
    }
    virtual error_state processSharedMemory() override { return error_state::SUCCESS; }
    virtual bool getShutdownFlag() override { return (SM_TM_!=nullptr) && (SM_TM_->shutdown!=0); }
    virtual void threadFunction() override {
        while (!getShutdownFlag()) {
            if (SM_TM_) { Monitor::Enter(SM_TM_->lockObject); try { SM_TM_->heartbeat |= bit_CONTROLLER; } finally { Monitor::Exit(SM_TM_->lockObject); } }
            Thread::Sleep(80);
        }
        System::Console::WriteLine("[Controller] thread exit.");
    }
private:
    SM_Lidar^ SM_L_; SM_GNSS^ SM_G_; SM_VehicleControl^ SM_VC_;
};





// VC.h
#pragma once
#include <UGVModule.h>
#include <SMObjects.h>
using namespace System;
using namespace System::Threading;

ref class VC : public UGVModule {
public:
    VC(SM_ThreadManagement^ sm_tm, SM_VehicleControl^ sm_vc) { SM_TM_ = sm_tm; SM_VC_ = sm_vc; }
    virtual error_state processSharedMemory() override { return error_state::SUCCESS; }
    virtual bool getShutdownFlag() override { return (SM_TM_!=nullptr) && (SM_TM_->shutdown!=0); }
    virtual void threadFunction() override {
        while (!getShutdownFlag()) {
            if (SM_TM_) { Monitor::Enter(SM_TM_->lockObject); try { SM_TM_->heartbeat |= bit_VC; } finally { Monitor::Exit(SM_TM_->lockObject); } }
            Thread::Sleep(100);
        }
        System::Console::WriteLine("[VC] thread exit.");
    }
private: SM_VehicleControl^ SM_VC_;
};


#pragma once
#include <UGVModule.h>
#include <SMObjects.h>

using namespace System;

ref class CrashAvoidance : public UGVModule {
public:
    CrashAvoidance(SM_ThreadManagement^ sm_tm, SM_Lidar^ sm_l, SM_VehicleControl^ sm_vc);

    virtual error_state processSharedMemory() override;
    virtual bool getShutdownFlag() override;
    virtual void threadFunction() override;

private:
    SM_Lidar^ SM_L_;
    SM_VehicleControl^ SM_VC_;
};




#include "CrashAvoidance.h"
using namespace System;
using namespace System::Threading;

CrashAvoidance::CrashAvoidance(SM_ThreadManagement^ sm_tm, SM_Lidar^ sm_l, SM_VehicleControl^ sm_vc) {
    SM_TM_ = sm_tm;
    SM_L_  = sm_l;
    SM_VC_ = sm_vc;
}

error_state CrashAvoidance::processSharedMemory() {
    return error_state::SUCCESS; // Week8 占位
}

bool CrashAvoidance::getShutdownFlag() {
    return (SM_TM_ != nullptr) && (SM_TM_->shutdown != 0);
}

void CrashAvoidance::threadFunction() {
    while (!getShutdownFlag()) {
        // 心跳置位（展示线程间通信）
        if (SM_TM_) {
            Monitor::Enter(SM_TM_->lockObject);
            try { SM_TM_->heartbeat |= bit_CRASHAVOIDANCE; }
            finally { Monitor::Exit(SM_TM_->lockObject); }
        }
        Thread::Sleep(90);
    }
    Console::WriteLine("[CrashAvoidance] thread exit.");
}








#pragma once
#include <NetworkedModule.h>
#include <SMObjects.h>

ref class LiDAR : public NetworkedModule {
public:
    LiDAR(SM_ThreadManagement^ sm_tm, SM_Lidar^ sm_l);

    // Week8 网络占位
    virtual error_state connect(String^ hostName, int portNumber) override;
    virtual error_state communicate() override;

    virtual error_state processSharedMemory() override;
    virtual bool getShutdownFlag() override;
    virtual void threadFunction() override;

private:
    SM_Lidar^ SM_L_;
    void writeScanToSharedMemory(const array<double>^ x, const array<double>^ y);
};



#include "LiDAR.h"
#include <cmath>
using namespace System;
using namespace System::Threading;

LiDAR::LiDAR(SM_ThreadManagement^ sm_tm, SM_Lidar^ sm_l) { SM_TM_ = sm_tm; SM_L_ = sm_l; }
error_state LiDAR::connect(String^, int){ return error_state::SUCCESS; }
error_state LiDAR::communicate(){ return error_state::SUCCESS; }
error_state LiDAR::processSharedMemory(){ return error_state::SUCCESS; }
bool LiDAR::getShutdownFlag(){ return (SM_TM_ != nullptr) && (SM_TM_->shutdown != 0); }

void LiDAR::writeScanToSharedMemory(const array<double>^ x, const array<double>^ y){
    if (!SM_L_) return;
    Monitor::Enter(SM_L_->lockObject);
    try {
        for (int i = 0; i < x->Length && i < SM_L_->x->Length; ++i) {
            SM_L_->x[i] = x[i];
            SM_L_->y[i] = y[i];
        }
    } finally { Monitor::Exit(SM_L_->lockObject); }
}

void LiDAR::threadFunction(){
    Console::WriteLine("[LiDAR] running — writing 361 points to SM and printing (x,y).");
    const int N = STANDARD_LIDAR_LENGTH; // 361
    array<double>^ x = gcnew array<double>(N);
    array<double>^ y = gcnew array<double>(N);

    double phase = 0.0;
    while (!getShutdownFlag()){
        phase += 0.05;
        for (int i = 0; i < N; ++i){
            double deg = (double)i;
            double rad = deg * 3.14159265358979323846 / 180.0;
            double r   = 5.0 + 2.0 * std::sin(2.0 * rad + phase); // 3~7 m 模拟
            x[i] = r * std::cos(rad);
            y[i] = r * std::sin(rad);
        }
        writeScanToSharedMemory(x, y);

        Console::WriteLine("LiDAR XY (361 pts):");
        for (int i = 0; i < N; ++i)
            Console::Write("( {0:F3}, {1:F3} ){2}", x[i], y[i], (i % 8 == 7) ? "\n" : "  ");
        if ((N % 8) != 0) Console::WriteLine();

        Thread::Sleep(50); // ~20Hz
    }
    Console::WriteLine("[LiDAR] thread exit.");
}




#pragma once
#include <UGVModule.h>
#include <SMObjects.h>

using namespace System;

ref class VC : public UGVModule {
public:
    VC(SM_ThreadManagement^ sm_tm, SM_VehicleControl^ sm_vc);

    virtual error_state processSharedMemory() override;
    virtual bool getShutdownFlag() override;
    virtual void threadFunction() override;

private:
    SM_VehicleControl^ SM_VC_;
};



#include "VC.h"
using namespace System;
using namespace System::Threading;

VC::VC(SM_ThreadManagement^ sm_tm, SM_VehicleControl^ sm_vc) {
    SM_TM_ = sm_tm;
    SM_VC_ = sm_vc;
}

error_state VC::processSharedMemory() {
    return error_state::SUCCESS; // Week 8 占位
}

bool VC::getShutdownFlag() {
    return (SM_TM_ != nullptr) && (SM_TM_->shutdown != 0);
}

void VC::threadFunction() {
    while (!getShutdownFlag()) {
        // 更新心跳位，表示线程在运行
        if (SM_TM_) {
            Monitor::Enter(SM_TM_->lockObject);
            try { SM_TM_->heartbeat |= bit_VC; }
            finally { Monitor::Exit(SM_TM_->lockObject); }
        }
        Thread::Sleep(100); // 10 Hz 刷新
    }
    Console::WriteLine("[VC] thread exit.");
}


#pragma once
#include <UGVModule.h>
#include <SMObjects.h>

using namespace System;

ref class Controller : public UGVModule {
public:
    Controller(SM_ThreadManagement^ sm_tm, SM_Lidar^ sm_l, SM_GNSS^ sm_g, SM_VehicleControl^ sm_vc);

    virtual error_state processSharedMemory() override;
    virtual bool getShutdownFlag() override;
    virtual void threadFunction() override;

private:
    SM_Lidar^          SM_L_;
    SM_GNSS^           SM_G_;
    SM_VehicleControl^ SM_VC_;
};



#include "Controller.h"
using namespace System;
using namespace System::Threading;

Controller::Controller(SM_ThreadManagement^ sm_tm, SM_Lidar^ sm_l, SM_GNSS^ sm_g, SM_VehicleControl^ sm_vc) {
    SM_TM_ = sm_tm;
    SM_L_  = sm_l;
    SM_G_  = sm_g;
    SM_VC_ = sm_vc;
}

error_state Controller::processSharedMemory() {
    return error_state::SUCCESS; // Week 8 占位
}

bool Controller::getShutdownFlag() {
    return (SM_TM_ != nullptr) && (SM_TM_->shutdown != 0);
}

void Controller::threadFunction() {
    while (!getShutdownFlag()) {
        // 心跳
        if (SM_TM_) {
            Monitor::Enter(SM_TM_->lockObject);
            try { SM_TM_->heartbeat |= bit_CONTROLLER; }
            finally { Monitor::Exit(SM_TM_->lockObject); }
        }
        Thread::Sleep(80);
    }
    Console::WriteLine("[Controller] thread exit.");
}





#pragma once
#include <UGVModule.h>
#include <SMObjects.h>

using namespace System;

ref class GNSS : public UGVModule {
public:
    GNSS(SM_ThreadManagement^ sm_tm, SM_GNSS^ sm_g);

    virtual error_state processSharedMemory() override;
    virtual bool getShutdownFlag() override;
    virtual void threadFunction() override;

private:
    SM_GNSS^ SM_G_;
};


#include "GNSS.h"
using namespace System;
using namespace System::Threading;

GNSS::GNSS(SM_ThreadManagement^ sm_tm, SM_GNSS^ sm_g) {
    SM_TM_ = sm_tm;
    SM_G_  = sm_g;
}

error_state GNSS::processSharedMemory() {
    // Week 8 只需返回成功，Week 9 可在此写 GNSS 数据
    return error_state::SUCCESS;
}

bool GNSS::getShutdownFlag() {
    return (SM_TM_ != nullptr) && (SM_TM_->shutdown != 0);
}

void GNSS::threadFunction() {
    Console::WriteLine("[GNSS] running...");
    while (!getShutdownFlag()) {
        // 设置心跳位，表示模块在运行
        if (SM_TM_) {
            Monitor::Enter(SM_TM_->lockObject);
            try { SM_TM_->heartbeat |= bit_GNSS; }
            finally { Monitor::Exit(SM_TM_->lockObject); }
        }
        Thread::Sleep(150); // 约 6~7 Hz 更新率
    }
    Console::WriteLine("[GNSS] thread exit.");
}






