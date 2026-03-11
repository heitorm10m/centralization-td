# cpp/AGENTS.md

## Escopo

Estas instruções valem para o núcleo C++ do solver.

## Regras do core

- Toda fórmula implementada deve ser dimensionalmente coerente.
- Toda hipótese estrutural deve ser documentada perto do código que a usa.
- Não misturar lógica de solver com serialização, CLI, YAML parsing ou plotting.
- Não usar nomes físicos fortes para grandezas que ainda são apenas `estimates`, `proxies` ou `reduced baselines`.
- Separar claramente:
  - geometria e frame local
  - discretização
  - montagem global
  - contato
  - centralizadores
  - torque & drag
  - acoplamento iterativo
  - pós-processamento
- Se um módulo crescer demais, quebrar em arquivos novos.

## Regras numéricas

- Preferir algoritmos robustos e simples antes de aumentar a sofisticação.
- Em problemas iterativos, documentar residual, critério de convergência e limite de iterações.
- Em contato penalizado, documentar o significado físico da penalidade e onde ela entra no residual.
- Em outputs, distinguir claramente vetor, magnitude, `estimate`, `reaction` e `force`.
- Manter convenções de sinais explícitas para perfis axiais, contato e torque.

## Direção técnica do solver

A direção do solver é:
modelo vetorial reduzido no frame local -> bow-by-bow -> torque tangencial vetorial -> calibração e validação.

Não voltar a novas simplificações escalares como direção principal.
