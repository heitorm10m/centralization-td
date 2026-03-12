# cpp/AGENTS.md

## Escopo

Estas instrucoes valem para o nucleo C++ do solver.

## Regras do core

- Toda formula implementada deve ser dimensionalmente coerente.
- Toda hipotese estrutural deve ser documentada perto do codigo que a usa.
- Nao misturar logica de solver com serializacao, CLI, YAML parsing ou plotting.
- Nao usar nomes fisicos fortes para grandezas que ainda sao apenas `estimates`, `proxies` ou `reduced baselines`.
- Explicitar se vetores e magnitudes estao no frame local da trajetoria ou em coordenadas transformadas.
- Separar claramente:
  - geometria e frame local
  - discretizacao
  - montagem global
  - contato
  - centralizadores
  - torque & drag
  - acoplamento iterativo
  - pos-processamento
- Se um modulo crescer demais, quebrar em arquivos novos.

## Regras numericas

- Preferir algoritmos robustos e simples antes de aumentar a sofisticacao.
- Em problemas iterativos, documentar residual, criterio de convergencia e limites de iteracao.
- Em contato penalizado, documentar o significado fisico da penalidade e onde ela entra no residual.
- Em outputs, distinguir claramente vetor, magnitude, `estimate`, `reaction` e `force`.
- Manter distinguiveis, no codigo e nos outputs, as contribuicoes do corpo da coluna e dos centralizadores para contato, atrito e torque.
- Manter convencoes de sinais explicitas para perfis axiais, contato e torque.

## Direcao tecnica do solver

A direcao do solver e:
modelo vetorial reduzido no frame local -> bow-by-bow -> torque tangencial vetorial -> calibracao/validacao.

Nao voltar a novas simplificacoes escalares como direcao principal.
