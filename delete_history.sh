git checkout "main"
git checkout --orphan "clean-main"
git add .
git commit -m "Reset history on main"
git branch -D "main"
git branch -m "main"

git push origin --force --all