# sailboat-control

## Installation

+ `cd ${HOME}`
+ `git clone --recursive https://github.com/D3473R/sailboat-control.git`
+ `cd sailboat-control`
+ `pipenv install`
+ `pipenv shell`
+ `MDEF=${HOME}/sailboat-control/sailboatcontrol/message_definitions/ pip install sailboatcontrol/pymavlink/ -v`
+ `exit`

Replace in `sailboatcontrol/pymavlink/generator/mavgen_typescript.py` every `node-mavlink` with `@ifrunistuttgart/node-mavlink`

## Generate new mavlink library files

### For [sailboat-control](https://github.com/D3473R/sailboat-control)

+ Edit `sailboatcontrol/message_definitions/v1.0/sailboat.xml`
+ Remove `sailboatcontrol/pymavlink/dialects/v20/sailboat.py`
+ Run `pipenv run python sailboatcontrol/control.py`

### For [sailboat-gcs](https://github.com/D3473R/sailboat-gcs)

+ Edit `sailboatcontrol/message_definitions/v1.0/sailboat.xml`
+ `mkdir mavlink`
+ `python sailboatcontrol/pymavlink/tools/mavgen.py -o ./mavlink --lang TypeScript --wire-protocol 2.0 sailboatcontrol/message_definitions/v1.0/sailboat.xml`
+ Move the `mavlink` folder to `sailboat-gcs/src/assets/`
