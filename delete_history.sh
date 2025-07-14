git checkout "dev"
git checkout --orphan "clean-dev"
git add .
git commit -m "Reset history on dev"
git branch -D "dev"
git branch -m "dev"

git checkout "main"
git checkout --orphan "clean-main"
git add .
git commit -m "Reset history on main"
git branch -D "main"
git branch -m "main"

git push origin --force --all