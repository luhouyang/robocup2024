# ROBOCUP


## Main Package Template

* https://github.com/joshnewans/my_bot

## Follow the Setup Notes

* [Setup Notes](https://github.com/luhouyang/robocup2024/blob/main/notes.md)

## VS Code Preferences (User JSON)

* yapf (EeyoreLee)

* XML (Red Hat)

```json
{
  "workbench.colorTheme": "Default High Contrast",
  "[python]": {
    "editor.formatOnSaveMode": "file",
    "editor.formatOnSave": true,
    "editor.defaultFormatter": "eeyore.yapf",
    "editor.formatOnType": false
  },
  "redhat.telemetry.enabled": true,
  "[xml]": {
    "editor.defaultFormatter": "redhat.vscode-xml",
    "editor.tabSize": 4,
    "editor.trimAutoWhitespace": true,
    "editor.formatOnSaveMode": "file",
    "editor.formatOnSave": true
  },
  "xml.format.maxLineWidth": 120,
}
```