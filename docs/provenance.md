# Подтверждение авторства

Как проверить, что этот код написан командой «Голуби» и когда он существовал.

## Что зафиксировано

| Что | Где | Что доказывает |
|---|---|---|
| История разработки | `git log` — коммиты с 16.09.2026 | код появлялся постепенно, по дням |
| Подписанные теги | `git tag -l 'submission-*'` | состояние кода подписано ключом автора |
| Отметка времени | `docs/provenance/*.txt` и `*.txt.ots` | хэш коммита записан в блокчейн Bitcoin через OpenTimestamps: код существовал не позже этой даты |
| Идентификатор работы | `GLB-K5-4660c47a8c6c` в LICENSE, README и заголовках исходных файлов | копию легко найти поиском |

## Как проверить

Отметку времени:

```bash
pip install opentimestamps-client
ots verify docs/provenance/submission-2026-09-29.txt.ots
```

Файл `submission-2026-09-29.txt` содержит хэш коммита и хэш дерева файлов. Их можно сверить:

```bash
git rev-parse submission-2026-09-29^{commit} submission-2026-09-29^{tree}
```

Подпись тега (открытый ключ автора — в `docs/provenance/allowed_signers`, он же опубликован на https://github.com/argunv.keys):

```bash
git -c gpg.ssh.allowedSignersFile=docs/provenance/allowed_signers tag -v submission-2026-09-29
```

## Если вы нашли копию

Напишите в Issues репозитория. Условия использования — в [LICENSE](../LICENSE).
