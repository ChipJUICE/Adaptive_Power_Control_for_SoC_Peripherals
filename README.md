# Adaptive Peripheral Power Management System

**PowerNap** is an adaptive, multi-peripheral **power management and clock-gating subsystem** designed and developed using the **CogniChip hardware design platform**.

The project demonstrates an end-to-end RTL design flow — from AI-assisted module generation to system-level simulation and synthesis — with a strong focus on **reducing switching activity and dynamic power** in SoC peripherals.

All modules were developed, integrated, simulated, and synthesized inside **CogniChip**, leveraging its OpenCOS-based simulation flow and Yosys-driven synthesis backend.

---

## 🚀 Key Highlights

- Developed using **CogniChip** (AI-assisted RTL generation)
- Modular, parameterized **SystemVerilog architecture**
- Runtime-programmable power management
- Fine-grained **clock gating** for power reduction
- Fully verified using **Verilator**
- Synthesized using **Yosys** (via CogniChip)
- Clean separation of RTL, simulation, and synthesis artifacts

---

## 🛠️ Toolchain & Platform

- **Design Platform:** CogniChip  
- **Simulation Framework:** OpenCOS  
- **Simulator:** Verilator  
- **Synthesis Engine:** Yosys  
- **Frontend / Parser:** Slang  
- **Waveform Format:** `.fst`  
- **Dependency Management:** `DEPS.yml`  

---

## 🧠 System Overview

The design targets an SoC-style environment with multiple peripherals exhibiting bursty and idle-heavy workloads.

Each peripheral is independently monitored and dynamically transitioned between power states to minimize unnecessary clock toggling and switching activity.

### Power States
- **ACTIVE** — Clock enabled, full operation
- **IDLE** — No recent activity, monitoring phase
- **SLEEP** — Clock gated, minimal switching activity

---

## 🧩 Module Descriptions

### 1️⃣ `cfg_regs` — Configuration & Control Registers   
Provides a programmable interface to control power behavior at runtime.

### 2️⃣ `activity_counter` — Activity Monitoring   
Tracks whether peripherals are active or idle.

### 3️⃣ `idle_predictor` — Adaptive Idle Logic   
Determines when a peripheral is eligible to enter SLEEP mode.

### 4️⃣ `power_fsm` — Power State Machine   
Controls the power state of each peripheral.

### 5️⃣ `clock_gater` — Clock Gating Logic   
Generates gated clocks for peripherals to reduce switching activity.

### 6️⃣ `perf_counters` — Performance Metrics   
Provides observability into power behavior.

### 7️⃣ `pwr_ctrl_top` — Top-Level Integration  
Integrates all submodules into a single SoC-ready block.

---

## 📁 Repository Structure

```text
CogniChip/
├── rtl/
├── Simulation/
├── Synthesis/
└── README.md
```

---

## ✨ Developed with CogniChip

This project showcases how **CogniChip** can be used to rapidly design, integrate, verify, and synthesize a complex, multi-module digital system while maintaining architectural clarity and correctness.

---

## 👥 Contributors

- **Naveen** — Idle Predictor, Power FSM, Performance Counters  
- **Sammy** — Configuration Registers, Activity Counter  
- **Armish** — Clock Gater  
