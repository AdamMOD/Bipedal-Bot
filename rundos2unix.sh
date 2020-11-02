# Removes pesky windows endlines
find ./src/ -type f -not -path '*/\.*' -exec grep -Il '.' {} \; | xargs -d '\n' -L 1 dos2unix -k
sleep 10