Push modifications to submodule

```bash
cd src/servers
git status
git add .
git commit -m "Fix/update in Semaphores"
git push origin servers
cd ../..
git add src/servers
git commit -m "Update submodule pointer to latest commit"
git push origin master 
```

Pull modifications from submodule:

```bash
cd src/servers
git checkout servers
git pull origin servers
cd ../..
git add src/servers
git commit -m "Update submodule pointer after pulling changes"
git push origin master
```