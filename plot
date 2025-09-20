if [ "$1" = "match" ]; then
  python3 py-scripts/match_frames.py
elif [ "$1" = "screen" ]; then
  python3 py-scripts/plot_2D.py
elif [ "$1" = "world" ]; then
  python3 py-scripts/plot_3D.py
else
  echo "Please provide correct args"
fi