# git使用

## 仓库

1. git取消init

   ` Rm -rf .git `

2. 克隆

   `  git clone https://github.com/flutter/flutter.git --depth 1 `不复制整个历史版本（防止文件太大下载失败）


## 提交

1. `git pull -u origin main`即保存远程链接，下一次直接`git push`就行了

## 版本回滚

1. 撤销修改（已经commit）

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

1. 怎么在已经有所有代码的文件中连接上指定git