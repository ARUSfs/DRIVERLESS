#!/bin/bash

if [[ ! -d $HOME/.venv/DV ]]; then
	python3 -m venv $HOME/.venv/DV
	echo "alias activate='. ~/.venv/DV/bin/activate'" >> $HOME/.bash_aliases
	echo "To activate the virtual environment, you can just type 'activate'.\nSimilarly, to deactivate just write 'deactivate'."
elif [[ ! -f "./requirements.txt" ]]; then
	echo "Script must be run from the repository's main directory"
	exit 1
fi

. ~/.venv/DV/bin/activate

pip3 install -r ./requirements.txt
echo "Updated requirements.txt dependencies."

deactivate
