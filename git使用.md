# git使用

## 仓库初始化

1. git取消init

   ` Rm -rf .git `

2. git remote add origin https://github.com/kz02-2021/training.git


## 提交

1. `git pull -u origin main`即保存远程链接，下一次直接`git push`就行了

## 版本回滚

1. 撤销修改（已经commit）**主要用这个**

   ```
   git reset HEAD .
   get checkout .
   ```
   
2. 撤销修改（已经add）

   ```
   git rm --cached . -r
   ```

3. 撤销修改（没有add）

   `git checkout .`

## 版本前滚

`git log` 显示所有版本

![1656775971181](C:\Users\DELL\AppData\Roaming\Typora\typora-user-images\1656775971181.png)

```
git reset --hard HEAD^ 	
回滚到上一个版本
git reset --hard HEAD^^
回滚到上两个版本
git reset --hard 55c49efab26d6a5c657d87e05b85……
回滚到指定版本
```

# 问题

1. 查看当前位置 ` git remote -v`
2. 