# `src/main.cpp` — Walkthrough

This document explains what the main `src/main.cpp` achieves for our robot.
Each section is in the same order as the file, top to bottom.
Our code is built on two main software libraries: **PROS** (an operating 
system for the Brain) and **LemLib** (a driving and odometry library).

---

## File header and attribution

https://github.com/The-Theory/Override-654T/blob/main/src/main.cpp#L1-L11

This comment block at the very top credits the coding team and gives general
information about our code's purpose and foundation. This is documentation only,
having no effect on how the robot. 

---

## Library imports

https://github.com/The-Theory/Override-654T/blob/main/src/main.cpp#L15-L18

Four lines that pull in outside code so this file can use it. The note 
`IWYU pragma: keep` on the LemLib line is an instruction to code-cleanup
tools telling them not delete that import automatically. 

---

