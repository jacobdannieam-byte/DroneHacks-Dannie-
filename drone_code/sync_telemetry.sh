#!/bin/bash
# Copies all flight records from drone 16 into the repository, in a form that Git can commit and
# GitHub can accept. Run it on the Mac, from anywhere:
#
#     drone_code/sync_telemetry.sh            copy and pack; commit nothing
#     drone_code/sync_telemetry.sh --commit   also commit the telemetry/ folder (only that folder)
#     drone_code/sync_telemetry.sh --push     also commit, then push the current branch
#     drone_code/sync_telemetry.sh --no-copy  pack what is already on the Mac; do not log in
#
# How it fits with the other scripts
# ----------------------------------
#   1. It runs collect_flight_records.sh. That script copies the raw records from the drone to
#      flight_records_from_drone/skybeta-16/. Git ignores that folder, because it is large
#      (750 MB on 2026-11-08) and it has camera pictures.
#   2. It packs each record file into telemetry/skybeta-16/, with the same folder names:
#        - Each text file (.csv, .json, .jsonl, .txt) is compressed with gzip. CSV files become
#          approximately ten times smaller. Read one with: gzip -dc file.csv.gz | head
#        - Camera pictures are not copied. They are too large for Git.
#        - A file that is still larger than LARGEST_PACKED_FILE_MEGABYTES after compression is
#          not copied. The script names it. GitHub refuses files over 100 MB.
#   3. With --commit or --push, it commits only the telemetry/ folder.
#
# Running it again packs only files that are new or that changed on the drone. gzip -n leaves
# the time out of the packed file, so a file that did not change gives the same bytes and Git
# sees no change.
#
# It never deletes anything, on the drone or on the Mac. It never arms the drone and never
# changes anything on the drone; the copy step only reads.
#
# The repository is public. The records hold no passwords or keys: they hold ROS reports, the
# drone's name and the ROS settings. Do not add any file that holds a secret to the records.
set -e
set -o pipefail  # A failure anywhere in a pipe counts as a failure.

# Our own choice. GitHub refuses files over 100 MB and warns over 50 MB, so stay below both.
LARGEST_PACKED_FILE_MEGABYTES=45

repository="$(cd "$(dirname "$0")/.." && pwd)"
raw_folder="$repository/flight_records_from_drone/skybeta-16"
packed_folder="$repository/telemetry/skybeta-16"

copy_from_drone=true
commit=false
push=false
for option in "$@"; do
    case "$option" in
        --no-copy) copy_from_drone=false ;;
        --commit) commit=true ;;
        --push) commit=true; push=true ;;
        *) echo "Unknown option '$option'. The options are --no-copy, --commit and --push."; exit 1 ;;
    esac
done

# --- 1. Copy the raw records from the drone --------------------------------------------------------
if [ "$copy_from_drone" = true ]; then
    "$repository/drone_code/collect_flight_records.sh"
fi
if [ ! -d "$raw_folder" ]; then
    echo "There are no records on the Mac in $raw_folder. Copy them first (run without --no-copy)."
    exit 1
fi

# --- 2. Pack each file into telemetry/ -------------------------------------------------------------
is_picture() {
    case "$1" in
        */pictures/*|*.jpg|*.jpeg|*.png|*.bmp|*.ppm|*.npy|*.npz) return 0 ;;
        *) return 1 ;;
    esac
}

packed_count=0
skipped_large=()
largest_packed_bytes=$((LARGEST_PACKED_FILE_MEGABYTES * 1024 * 1024))

while IFS= read -r -d '' raw_file; do
    relative_path="${raw_file#"$raw_folder"/}"
    if is_picture "$relative_path"; then
        continue
    fi
    packed_file="$packed_folder/$relative_path.gz"
    # Skip a file that was packed after its last change.
    if [ -f "$packed_file" ] && [ "$packed_file" -nt "$raw_file" ]; then
        continue
    fi
    mkdir -p "$(dirname "$packed_file")"
    gzip -n -9 -c "$raw_file" > "$packed_file.partial"
    packed_bytes=$(wc -c < "$packed_file.partial")
    if [ "$packed_bytes" -gt "$largest_packed_bytes" ]; then
        rm -f "$packed_file.partial"
        skipped_large+=("$relative_path")
        continue
    fi
    mv "$packed_file.partial" "$packed_file"
    packed_count=$((packed_count + 1))
done < <(find "$raw_folder" -type f ! -name '.*' ! -name COLCON_IGNORE -print0)

echo "Packed $packed_count new or changed files into $packed_folder"
echo "The packed telemetry now takes $(du -sh "$packed_folder" | cut -f1)."
for path in "${skipped_large[@]}"; do
    echo "NOT COPIED (over $LARGEST_PACKED_FILE_MEGABYTES MB after compression): $path"
done

# --- 3. Commit and push, if asked ----------------------------------------------------------------
if [ "$commit" = false ]; then
    echo "Nothing was committed. To commit: drone_code/sync_telemetry.sh --no-copy --commit"
    exit 0
fi
cd "$repository"
git add -- telemetry
if git diff --cached --quiet -- telemetry; then
    echo "No new telemetry to commit."
else
    # Only the telemetry/ folder goes into this commit, even if other files are staged.
    git commit -m "Add drone 16 telemetry" -- telemetry
fi
if [ "$push" = true ]; then
    git push
fi
