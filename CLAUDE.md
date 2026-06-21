# edubot_navigation — Claude guidelines

Navigation for EduBot: Nav2 and SLAM launch files, parameters and maps.

This is an **ament_cmake** package; its only Python is the launch files. It
follows the EduBot conventions from the reference repo `edubot_hardware`, with a
**lean** tooling subset: no Python package, so no pytest/coverage and **no colcon
build in CI** — just linting, hooks and versioning.

## Language

Everything is written in **English** — code, comments, launch files, READMEs,
commit messages, config. This holds even when a chat with the maintainer is in
another language.

## Naming: OmniBot → EduBot

The project was formerly called **OmniBot**; it is now **EduBot**. Always use
`EduBot`/`edubot`. Fix any `OmniBot`/`omnibot` leftovers.

## Commits

All commits MUST follow [Conventional Commits](https://www.conventionalcommits.org),
enforced by the `commitizen` commit-msg hook. `<type>(<scope>): <summary>` —
imperative, lower case, no trailing period, summary < ~72 chars.

## Metadata

- Maintainer / contact: **Vectoral**, **info@vectoral.ch** (in `package.xml`).
- License: **PolyForm Perimeter 1.0.0** — keep `package.xml`, `LICENSE`, README consistent.

## Tooling

No uv project here. Tools run via `uvx` and pre-commit:

```bash
uvx pre-commit install --install-hooks          # git hooks (once per clone)
uvx pre-commit install --hook-type commit-msg
uvx ruff@0.12.0 check --fix . && uvx ruff@0.12.0 format .   # before pushing
```

- ruff config lives in `pyproject.toml` (`[tool.ruff]`, rules
  `E,F,W,I,B,UP,SIM,RUF`, line length 99).
- CI runs ruff (lint + format) and commit-message linting on PRs — nothing else.
- `commitizen` bumps the version in `package.xml` and generates `CHANGELOG.md`
  at release time (`uvx commitizen bump`).
