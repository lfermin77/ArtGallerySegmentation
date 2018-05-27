for f in *.poly
do
    echo $f
    ../../dude2d -j $f
done
if [ ! -d "json" ]; then
    mkdir json
fi
mv *.json json/
