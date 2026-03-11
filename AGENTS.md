# AGENTS.md

## Objetivo do projeto

Este repositório implementa um software científico de centralização de casing e torque & drag, com foco em evolução progressiva para uma arquitetura próxima da classe stiff-string 3D vetorial usada na literatura moderna e em softwares comerciais avançados.

## Estado atual e direção

O projeto já evoluiu por fases:

- bootstrap e build híbrido C++ + Python
- geometria 3D aproximada da trajetória e modelo de dados
- baseline mecânico com `EI`, peso submerso e curvatura
- contato local simplificado
- solver global reduzido ao longo do MD
- torque & drag reduzido
- acoplamento iterativo entre lateral/contact e T&D
- solver vetorial reduzido no frame local da trajetória
- centralizador bow-spring detalhado, bow por bow, com resultante vetorial e contribuição reduzida ao torque

A direção final desejada é:

1. solver estrutural vetorial no frame local, cada vez mais próximo de um stiff-string 3D fisicamente defensável
2. centralizador bow-spring detalhado, bow por bow, com parametrização calibrável
3. torque tangencial vetorial derivado da resultante dos bows e da interação com o anular
4. validação, calibração e comparação com literatura, benchmarks e casos de referência
5. maturidade gradual em direção a um software técnico robusto, sem fingir equivalência precoce a produto comercial

## Regras permanentes de arquitetura

- Núcleo numérico em C++.
- Interface, runner, CLI, YAML I/O e pós-processamento em Python.
- Unidades internas sempre em SI.
- Não hardcodar casos de poço, strings ou layouts de centralizadores dentro do solver.
- Não misturar plotting, CLI, serialização ou documentação com o kernel numérico.
- Toda alteração de física deve vir com hipótese explícita.
- Toda nova grandeza física deve ter unidade clara no código, testes ou documentação.
- Toda alteração no solver deve vir acompanhada de testes.
- Toda aproximação provisória deve ser nomeada honestamente como `reduced`, `proxy`, `estimate`, `baseline` ou equivalente.
- Não renomear outputs provisórios como se fossem solução final.
- Manter YAMLs legíveis, versionados e externos ao solver.

## Regras de modelagem física

- Não rebaixar o projeto para novas simplificações escalares se a etapa atual já exige formulação vetorial.
- A transição prioritária do projeto é:
  solver vetorial 3D reduzido -> bow-by-bow -> torque tangencial vetorial -> calibração/validação.
- O modelo atual não deve ser descrito como stiff-string 3D completo enquanto não houver 6 GDL por nó, contato/fricção 3D completos e modelagem final de centralizador.
- Em fases reduzidas, preferir honestidade física a aparência de completude.
- Quando uma grandeza ainda não puder ser defendida fisicamente, preferir `null`, `status` explícito ou nomenclatura provisória a inventar valor.
- Não substituir a direção stiff-string por um soft-string como solução final do projeto.
- Documentar hipóteses geométricas, constitutivas e numéricas explicitamente.

## Regras de implementação

- Antes de implementar, inspecionar a arquitetura existente.
- Preservar build, bindings, testes e CLI.
- Fazer mudanças pequenas e coerentes.
- Criar novos arquivos quando necessário em vez de inflar arquivos antigos.
- Atualizar README e roadmap quando a arquitetura física ou o contrato do solver mudar.
- Não mascarar limitações numéricas ou físicas.
- Separar claramente geometria, discretização, solver, contato, centralizadores, torque & drag, acoplamento e pós-processamento.

## Critérios mínimos antes de encerrar uma fase

- build C++ passa
- bindings `pybind11` passam
- pacote Python instala
- `pytest` passa
- `ctest` passa
- CLI `summary` funciona
- CLI `run-stub` funciona
- documentação da fase foi atualizada
- o que ainda NAO foi implementado está explicitado

## Roadmap técnico prioritário

As próximas grandes etapas devem seguir esta lógica:

1. integrar bow-spring detalhado cada vez mais diretamente ao solver vetorial
2. evoluir o torque tangencial vetorial baseado na resultante dos bows
3. refinar contato e fricção no anular
4. calibrar e validar contra literatura e benchmarks
5. amadurecer recursos de engenharia próximos de software comercial

## O que evitar

- pular diretamente para "produto final"
- fingir equivalência a software comercial proprietário
- esconder aproximações físicas
- quebrar compatibilidade sem necessidade
- introduzir complexidade 3D total sem ganho físico claro
- misturar documentação aspiracional com funcionalidade realmente implementada
