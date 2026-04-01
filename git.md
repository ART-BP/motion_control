cd /你的/项目路径
git init
git config --global user.name "你的名字"
git config --global user.email "你的邮箱"
git add .
git commit -m "初次提交"
git branch -M main
git remote add origin 仓库地址
git push -u origin main


# 提交
git add .
git commit --amend
git push -u origin main --force-with-lease


# 修改本地与远程连接
git remote -v
git remote set-url origin https://github.com/ART-BP/motion_control.git

# 拉取远程
# 拉取远程main分支
git pull origin main
