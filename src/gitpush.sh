git add .
echo 'Enter commit message:'
read commitMessage
git commit -m "$commitMessage"

echo 'enter the branch name:'

read branch
git push origin $branch
read