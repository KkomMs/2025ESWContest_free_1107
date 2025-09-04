# ☕ [BArista RObot: BARO]

## 📑 프로젝트 개요

- **BARO**
    
    SCARA 구조 기반 모듈형 바리스타 로봇 시스템
    
- **제작 목적**
    
    시중 바리스타 로봇 시스템에 비해 저렴한 비용 및 모듈형 구조를 통한 유지보수 효율성을 높여, 예비 창업자들의 진입장벽을 낮추고 커피 산업 내 자동화 기술의 대중화를 목표로 함.
    
- **작품 외관**
  
<img width="780" height="597" alt="Image" src="https://github.com/user-attachments/assets/341845cc-1f50-42e8-bf20-e12e5a97dcab" />
    
- **시연 영상(사진 클릭 시 재생)**

[![BARO](https://github.com/user-attachments/assets/341845cc-1f50-42e8-bf20-e12e5a97dcab)](https://youtu.be/kkzzoTA4NTc?si=MwrUqrTWJpY9Vvwy)
    

## 🧑‍💻 Team.HMH
**🤖 [한양대학교 ERICA 로봇공학과]**
| 팀장 김영성 | 팀원 강범진 | 팀원 김하영 | 팀원 정진우 |
| --- | --- | --- | --- |
| <div align="center"><img src="https://github.com/user-attachments/assets/178a1d4b-0fff-4015-a2bf-edf339d14269" width="100"/></div> | <div align="center"><img src="https://github.com/user-attachments/assets/60da614f-2460-4507-a8c4-d1108d3da024" width="100"/></div> | <div align="center"><img src="https://github.com/user-attachments/assets/6755228b-f110-4001-a627-ba0e8c4c169a" width="100"/></div> | <div align="center"><img src="https://github.com/user-attachments/assets/ac557dfb-49c3-4fc8-86c7-36cc69cf3029" width="100"/></div> |
| <div align="center">전장 시스템 설계, 통신</div> | <div align="center">로봇 및 플랫폼 설계</div> | <div align="center">로봇 제어</div> | <div align="center">로봇 제어</div> |
***
## ⚙️ HW

### 🛠️ 로봇 및 플랫폼
<img width="410" height="597" alt="Image" src="https://github.com/user-attachments/assets/3e579550-8bc1-4c30-b9fc-a0d142accca6" />
<img width="290" height="600" alt="Image" src="https://github.com/user-attachments/assets/ba4fc58c-203c-43f6-8768-4a17b5d36704" />

- **모듈형 설계**
  
    모듈형 설계(Base, 1st link, 2nd link, gripper)를 통한 빠른 부품 교환 가능. 유지보수 용이

<img width="620" height="418" alt="Image" src="https://github.com/user-attachments/assets/6b529a51-c30b-4ab1-9635-30e9811ec182" />

- **로봇 외관**
  
    주변 장비(제빙기, 커피 머신 등)의 통합 관리 가능


***
## 💻 SW

### 🗂️ 디렉토리 구조

<aside>

```
├── Arduino
│   └── main.cpp
├── motor_control
│   ├── include
│   │   ├── CAN_Access.h
│   │   ├── combined_arduinos.h
│   │   ├── combined_dxl.h
│   │   └── motor_controller.h
│   ├── integrated_control
│   │   └── integrated_control.cpp    # robot testing code
│   ├── src
│   │   ├── combined_arduinos.cpp
│   │   ├── combined_dxl.cpp
│   │   └── motor_controller.cpp
│   └── trajectory
│       └── trajectory.cpp    # simulation
└── README.md
```

</aside>

### 💡 알고리즘
- **전체적인 흐름도**
<div align="center">
    <img width="599" alt="Image" src="https://github.com/user-attachments/assets/2fbdcfb6-f4d4-4e56-a6fb-6ee0474732a9" />
</div>

- **음료 제작 (Ice Americano, Hot Americano, Ice Tea, Ice Tea Americano)**
<div align="center">
    <img width="402" height="585" alt="Image" src="https://github.com/user-attachments/assets/927f88aa-7d69-47c3-81d2-826578aa9d97" />   
    <img width="402" height="549" alt="Image" src="https://github.com/user-attachments/assets/372f6fee-9a01-4c5d-9cab-996c99db0ec3" />
</div>

<div align="center">
    <img width="317" height="585" alt="Image" src="https://github.com/user-attachments/assets/6d3421e6-362d-4b52-a50a-38411d10d759" />   
    <img width="500" height="485" alt="Image" src="https://github.com/user-attachments/assets/b518ec97-865f-48dd-9b80-be194ca4a34f" />
</div>

***

## 🔧 Tools
![Arduino](https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=Arduino&logoColor=white)
![VS](https://img.shields.io/badge/Visual_Studio-5C2D91?style=for-the-badge&logo=visual%20studio&logoColor=white)
![VSC](https://img.shields.io/badge/Visual_Studio_Code-0078D4?style=for-the-badge&logo=visual%20studio%20code&logoColor=white)   
![C](https://img.shields.io/badge/C-00599C?style=for-the-badge&logo=c&logoColor=white)
![CPP](https://img.shields.io/badge/C%2B%2B-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)
