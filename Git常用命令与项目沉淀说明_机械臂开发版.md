# Git 常用命令与项目沉淀说明

机械臂开发版 / BionicArm_Control


# 0. 你现在只需要记住的核心流程
Git 初学阶段不要一上来学太多复杂概念。日常开发先固定一个最小闭环：看状态、保存修改、提交、上传。
```powershell
git status
git add .
git commit -m "说明这次改了什么"
git push
```

| 动作 | 命令 | 什么时候用 |
| --- | --- | --- |
| 看当前改了什么 | git status | 每次开始、提交前、提交后都看 |
| 查看具体差异 | git diff | 确认自己到底改了哪些代码 |
| 加入暂存区 | git add . | 准备把当前修改纳入下一次提交 |
| 提交到本地仓库 | git commit -m "..." | 形成一个可回滚的版本点 |
| 上传到 GitHub | git push | 把本地提交同步到云端 |
| 从 GitHub 拉取 | git pull | 工控机或另一台电脑同步最新代码 |


# 1. Git / GitHub / 仓库的关系
| 概念 | 含义 | 在你项目里的例子 |
| --- | --- | --- |
| Git | 本地版本管理工具 | 记录机械臂代码每次修改 |
| GitHub | 云端代码托管平台 | 备份代码、给工控机拉取、给同事查看 |
| Repository / 仓库 | 一个项目的代码库 | BionicArm_Control |
| Commit / 提交 | 一次明确的版本记录 | 修复 Normal 帧 DLC 后提交一次 |
| Branch / 分支 | 并行开发线 | 初期只用 main，后期再加 feature 分支 |
| Tag / 标签 | 重要版本标记 | v0.1-baseline-advanced-and-dualcan-v3 |


# 2. 第一次配置 Git
在 Windows PowerShell 里先配置用户名和邮箱。这只需要做一次。
```powershell
git config --global user.name "你的GitHub用户名"
git config --global user.email "你的GitHub邮箱"

git config --global --list
```

推荐打开长路径支持，避免 Windows 下路径过长导致问题。
```powershell
git config --global core.longpaths true
```


# 3. 针对当前机械臂项目的最简仓库结构
当前阶段先不要把目录做复杂。只上传两个有价值版本：Advanced 单 CAN 版本、Dual CAN V3 可变 DLC 版本。
```
BionicArm_Control/
├── README.md
├── .gitignore
├── legacy/
│   └── advanced_single_can/
│       ├── arm_cli_menu.py
│       ├── can_motor_arm_lib.py
│       ├── example_single_arm.py
│       ├── motor_test.py
│       └── usb2can_demo.py
└── stable/
    └── dualcan_v3/
        ├── dualcan_arm_control_lib_v3.py
        ├── dualcan_dualarm_menu_v3.py
        ├── test_single_motor_channel_v3.py
        └── README_dualcan_dualarm_v3.md
```

- legacy/advanced_single_can：保留早期已验证的 Advanced 单 CAN 版本，用于回溯和对比。
- stable/dualcan_v3：当前主线版本，支持同一个 COM 口下 CAN1/CAN2 分别控制左右臂。
- 双can_arm_v1 这类已知有 DLC 问题的版本先不要上传，避免误用。


# 4. 从零创建本地仓库并第一次提交

## 4.1 新建干净目录
```powershell
cd C:\Users\lzy\Desktop
mkdir BionicArm_Control_Git
cd BionicArm_Control_Git

mkdir legacy
mkdir stable
mkdir legacy\advanced_single_can
mkdir stable\dualcan_v3
```


## 4.2 复制需要沉淀的两个版本
```powershell
$src = "C:\Users\lzy\Desktop\BionicArm_Control"
$repo = "C:\Users\lzy\Desktop\BionicArm_Control_Git"

Copy-Item "$src\工控机_arm\arm_cli_menu.py" "$repo\legacy\advanced_single_can\"
Copy-Item "$src\工控机_arm\can_motor_arm_lib.py" "$repo\legacy\advanced_single_can\"
Copy-Item "$src\工控机_arm\example_single_arm.py" "$repo\legacy\advanced_single_can\"
Copy-Item "$src\工控机_arm\motor_test.py" "$repo\legacy\advanced_single_can\"
Copy-Item "$src\工控机_arm\usb2can_demo.py" "$repo\legacy\advanced_single_can\"

Copy-Item "$src\双can_arm_v2\dualcan_arm_control_lib_v3.py" "$repo\stable\dualcan_v3\"
Copy-Item "$src\双can_arm_v2\dualcan_dualarm_menu_v3.py" "$repo\stable\dualcan_v3\"
Copy-Item "$src\双can_arm_v2\test_single_motor_channel_v3.py" "$repo\stable\dualcan_v3\"
Copy-Item "$src\双can_arm_v2\README_dualcan_dualarm_v3.md" "$repo\stable\dualcan_v3\"
```

如果你的实际目录不是 双can_arm_v2，就把命令里的目录名改成你本地真实目录。

## 4.3 创建 .gitignore
```powershell
@"
__pycache__/
*.pyc
*.pyo
*.pyd

.venv/
venv/
env/

*.log
logs/

.vscode/
.idea/

.DS_Store
Thumbs.db
"@ | Set-Content -Encoding UTF8 .gitignore
```


## 4.4 初始化并提交
```powershell
git init
git status

git add .
git commit -m "chore: import advanced single CAN and dual CAN v3 baseline"
```


# 5. 连接 GitHub 并上传
1. 打开 GitHub，新建仓库 BionicArm_Control。
1. Visibility 建议先选 Private。
1. 不要勾选 README、.gitignore、license，因为本地已经有。
1. 复制 GitHub 给出的 HTTPS 仓库地址。

```powershell
git branch -M main
git remote add origin https://github.com/你的用户名/BionicArm_Control.git
git push -u origin main
```

第一次 push 可能弹出浏览器登录 GitHub，按提示授权。

# 6. 给重要版本打标签
Tag 用来标记一个里程碑。当前建议把 Advanced 单 CAN + Dual CAN V3 这次沉淀打成 v0.1。
```powershell
git tag v0.1-baseline-advanced-and-dualcan-v3
git push origin v0.1-baseline-advanced-and-dualcan-v3

git tag
```

以后发现问题想回到这个版本，可以用 tag 定位。

# 7. 日常开发最常用命令

## 7.1 查看状态
```powershell
git status
```

红色通常表示还没有加入暂存区，绿色表示已经 add，准备 commit。

## 7.2 查看改了什么
```powershell
git diff

git diff 文件名.py
```

提交前一定建议看一眼 diff，避免把临时测试代码、错误参数提交上去。

## 7.3 暂存修改
```powershell
git add .

git add stable/dualcan_v3/dualcan_arm_control_lib_v3.py
```

初期可以多用 git add .；后期熟悉后，建议按文件 add。

## 7.4 提交修改
```powershell
git commit -m "fix: use variable length normal frame for dual CAN"
```

| 提交类型 | 含义 | 例子 |
| --- | --- | --- |
| feat | 新增功能 | feat: add dual arm api facade |
| fix | 修复问题 | fix: correct C1 DLC length |
| docs | 文档修改 | docs: add git command guide |
| test | 测试相关 | test: add single motor channel test |
| refactor | 重构代码但不改变功能 | refactor: split motor protocol module |
| chore | 杂项、初始化、配置 | chore: import baseline code |


## 7.5 上传到 GitHub
```powershell
git push
```


## 7.6 从 GitHub 拉取最新代码
```powershell
git pull
```

在工控机或另一台电脑开始改代码前，先执行 git pull。

# 8. 在 Ubuntu i5 工控机上拉取和使用
```bash
cd ~
git clone https://github.com/你的用户名/BionicArm_Control.git
cd BionicArm_Control/stable/dualcan_v3
pip3 install pyserial

python3 test_single_motor_channel_v3.py --port /dev/ttyACM0 --channel 1 --motor-id 34 --no-move --debug
```

如果串口权限不够，执行：
```bash
sudo usermod -aG dialout $USER
# 然后注销重新登录，或者重启
```


# 9. 常见场景怎么操作

## 9.1 我改了代码，想保存一个版本
```powershell
git status
git diff
git add .
git commit -m "fix: describe what changed"
git push
```


## 9.2 我只是想看历史提交
```powershell
git log --oneline --graph --decorate --all

git log --oneline -10
```


## 9.3 我想看某次提交改了什么
```powershell
git show 提交ID

git show --stat 提交ID
```


## 9.4 我改错了一个文件，想恢复到上次提交
危险程度较低：只恢复某个文件。
```powershell
git restore 文件名.py
```

恢复所有未提交修改，危险，慎用。
```powershell
git restore .
```


## 9.5 我已经 git add 了，但不想暂存
```powershell
git restore --staged 文件名.py

git restore --staged .
```

这不会删掉你的修改，只是把绿色暂存状态变回红色未暂存状态。

## 9.6 我想临时保存现场，切去做别的
```powershell
git stash push -m "temp: testing dual CAN menu"

git stash list

git stash pop
```

stash 适合“当前改到一半，不想提交，但又要切换现场”的情况。

## 9.7 我想删除 GitHub 上不该上传的缓存文件
```powershell
git rm -r --cached __pycache__
git commit -m "chore: remove cached python bytecode"
git push
```

如果文件已经被 Git 跟踪，后来加 .gitignore 不会自动删除，需要用 git rm --cached。

# 10. 分支：初期可以不用，但要知道怎么用
你现在刚入门，主线开发先都放在 main 可以。但后面做 API v1 或实时线程时，建议用分支。
| 动作 | 命令 |
| --- | --- |
| 创建并切换新分支 | git checkout -b feature/api-v1 |
| 查看分支 | git branch |
| 切回 main | git checkout main |
| 把分支合并到 main | git merge feature/api-v1 |
| 删除本地分支 | git branch -d feature/api-v1 |

```powershell
git checkout -b feature/api-v1
# 修改代码
git add .
git commit -m "feat: add dual arm api v1"
git push -u origin feature/api-v1
```


# 11. 冲突处理：看到 conflict 不要慌
如果你和工控机、同事同时改了同一个文件，git pull 可能出现冲突。文件里会出现：
```
<<<<<<< HEAD
你本地的内容
=======
远端的内容
>>>>>>> origin/main
```

1. 打开冲突文件，手动保留正确内容。
1. 删除 <<<<<<<、=======、>>>>>>> 这些标记。
1. 保存文件。
1. 执行 git add 文件名。
1. 执行 git commit。
1. 执行 git push。


# 12. 回滚与撤销：新手优先用安全方式
| 目标 | 推荐命令 | 说明 |
| --- | --- | --- |
| 撤销未提交的某个文件 | git restore 文件名 | 安全常用 |
| 取消暂存 | git restore --staged 文件名 | 不删除修改 |
| 回退到某个历史版本查看 | git checkout 标签或提交ID | 只查看，不建议在这里直接开发 |
| 生成一个反向提交 | git revert 提交ID | 适合已经 push 的提交 |
| 强行回退历史 | git reset --hard 提交ID | 危险，初期尽量不要用 |

已经 push 到 GitHub 的提交，不建议新手用 reset --hard 改历史，优先用 git revert。

# 13. 机械臂项目推荐的版本命名
| 版本 | 含义 |
| --- | --- |
| v0.1-baseline-advanced-and-dualcan-v3 | Advanced 单 CAN + Dual CAN V3 基线 |
| v0.2-menu-test-report | 菜单功能测试完成 |
| v0.3-api-v1 | 上层稳定 API 第一版 |
| v0.4-module-split | 底层模块拆分 |
| v0.5-state-cache | 异步状态缓存 |
| v1.0-algorithm-stable-api | 交付算法组稳定版 |

```powershell
git tag v0.2-menu-test-report
git push origin v0.2-menu-test-report
```


# 14. 上传前检查清单
- 运行 git status，确认没有不该提交的临时文件。
- 运行 git diff，确认修改内容是自己想提交的。
- 不要提交 __pycache__、.pyc、.venv、日志、截图、临时压缩包。
- README 里至少写清楚怎么运行单电机测试和菜单。
- 重要版本打 tag。
- 能跑通的版本才放 stable；有问题但有参考价值的版本放 legacy；明显错误版本先不上传。


# 15. Git 命令速查表
| 目的 | 命令 |
| --- | --- |
| 查看状态 | git status |
| 查看差异 | git diff |
| 暂存全部 | git add . |
| 暂存指定文件 | git add 文件名 |
| 提交 | git commit -m "说明" |
| 上传 | git push |
| 拉取 | git pull |
| 克隆仓库 | git clone 仓库地址 |
| 查看历史 | git log --oneline --graph --decorate --all |
| 查看远端 | git remote -v |
| 添加远端 | git remote add origin 仓库地址 |
| 改主分支名 | git branch -M main |
| 创建标签 | git tag 标签名 |
| 推送标签 | git push origin 标签名 |
| 查看标签 | git tag |
| 撤销未提交修改 | git restore 文件名 |
| 取消暂存 | git restore --staged 文件名 |
| 临时保存现场 | git stash push -m "说明" |
| 恢复 stash | git stash pop |


# 16. 给你的机械臂项目的建议
现阶段最重要的不是把 Git 学复杂，而是形成固定动作：每个能跑通的版本都提交，每个里程碑都打标签，每次代码给工控机使用都能从 GitHub 拉取。
- Advanced 单 CAN 版本作为 legacy 留档。
- Dual CAN V3 作为 stable 当前主线。
- 菜单功能测试完成后再开始包装 dualcan_dualarm_api_v1.py。
- API v1 做完后，菜单也改成调用 API，这样菜单就是 API 的测试入口。
- 等 API 稳定后，再做异步状态缓存和实时轨迹流。

