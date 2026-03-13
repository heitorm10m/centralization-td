# AGENTS.md

## Objetivo do projeto

Este repositorio implementa um software cientifico de centralizacao de casing e torque & drag, com foco em evolucao progressiva para uma arquitetura proxima da classe stiff-string 3D vetorial usada na literatura moderna e em softwares comerciais avancados.

## Estado atual e direcao

O projeto ja evoluiu por fases:

- bootstrap e build hibrido C++ + Python
- geometria 3D aproximada da trajetoria e modelo de dados
- baseline mecanico com `EI`, peso submerso e curvatura
- contato local simplificado
- solver global reduzido ao longo do MD
- torque & drag reduzido
- acoplamento iterativo entre lateral/contact e T&D
- solver vetorial reduzido no frame local da trajetoria
- centralizador bow-spring detalhado, bow por bow, com resultante vetorial e contribuicao reduzida ao torque
- benchmark suite e calibracao interna inicial para o modelo atual

A direcao final desejada e:

1. solver estrutural vetorial no frame local, cada vez mais proximo de um stiff-string 3D fisicamente defensavel
2. centralizador bow-spring detalhado, bow por bow, integrado de forma cada vez mais direta ao solver
3. torque tangencial vetorial derivado da resultante dos bows e da interacao com o anular
4. refinamento progressivo de contato e friccao no anular, ainda dentro da direcao vetorial/local-frame
5. calibracao, validacao e comparacao com literatura, benchmarks e casos de referencia, sem claims prematuros

## Visao de longo prazo do produto

Apos o fechamento da fisica principal e da validacao/calibracao, o repositorio deve evoluir gradualmente para um software tecnico de nivel comercial, com:

- banco de dados de centralizadores e materiais
- importacao de casos reais
- comparacao automatica entre cenarios
- relatorios profissionais
- rastreabilidade de resultados
- recursos de engenharia voltados a uso por clientes

Esses recursos devem ser adicionados somente depois que a fisica principal estiver suficientemente estavel e validada.

## Estado da validacao

- O projeto ja possui uma fase inicial de benchmark/calibracao interna.
- Benchmark/calibracao interna servem para coerencia, tendencias e ajuste inicial de parametros; nao substituem validacao quantitativa externa.
- Ainda nao existe validacao quantitativa externa suficiente para alegar equivalencia com literatura completa ou software comercial.

## Regras permanentes de arquitetura

- Nucleo numerico em C++.
- Interface, runner, CLI, YAML I/O e pos-processamento em Python.
- Unidades internas sempre em SI.
- Nao hardcodar casos de poco, strings ou layouts de centralizadores dentro do solver.
- Nao misturar plotting, CLI, serializacao, YAML parsing ou documentacao com o kernel numerico.
- Toda alteracao de fisica deve vir com hipotese explicita.
- Toda nova grandeza fisica deve ter unidade clara no codigo, testes ou documentacao.
- Toda alteracao no solver deve vir acompanhada de testes.
- Toda aproximacao provisoria deve ser nomeada honestamente como `reduced`, `proxy`, `estimate`, `baseline` ou equivalente.
- Nao renomear outputs provisorios como se fossem solucao final.
- Manter YAMLs legiveis, versionados e externos ao solver.

## Regras de modelagem fisica

- Nao rebaixar o projeto para novas simplificacoes escalares se a etapa atual ja exige formulacao vetorial.
- A transicao prioritaria do projeto e:
  solver vetorial reduzido no frame local -> bow-by-bow -> torque tangencial vetorial -> refinamento de contato/friccao no anular -> calibracao/validacao.
- O modelo atual nao deve ser descrito como stiff-string 3D completo enquanto nao houver 6 GDL por no, contato/friccao 3D completos e modelagem final de centralizador.
- Em fases reduzidas, preferir honestidade fisica a aparencia de completude.
- Quando uma grandeza ainda nao puder ser defendida fisicamente, preferir `null`, `status` explicito ou nomenclatura provisoria a inventar valor.
- Nao substituir a direcao stiff-string por um soft-string como solucao final do projeto.
- Documentar hipoteses geometricas, constitutivas e numericas explicitamente.

## Regras de implementacao

- Antes de implementar, inspecionar a arquitetura existente.
- Preservar build, bindings, testes e CLI.
- Fazer mudancas pequenas e coerentes.
- Criar novos arquivos quando necessario em vez de inflar arquivos antigos.
- Atualizar README e roadmap quando a arquitetura fisica ou o contrato do solver mudar.
- Nao mascarar limitacoes numericas ou fisicas.
- Separar claramente geometria, discretizacao, solver, contato, centralizadores, torque & drag, acoplamento e pos-processamento.

## Criterios minimos antes de encerrar uma fase

- build C++ passa
- bindings `pybind11` passam
- pacote Python instala
- `pytest` passa
- `ctest` passa
- CLI `summary` funciona
- CLI `run-stub` funciona
- documentacao da fase foi atualizada
- o que ainda NAO foi implementado esta explicitado

## Roadmap tecnico prioritario

As proximas grandes etapas devem seguir esta logica:

1. integrar bow-spring detalhado cada vez mais diretamente ao solver vetorial
2. evoluir o torque tangencial vetorial baseado na resultante dos bows
3. refinar contato e friccao no anular
4. calibrar e validar contra literatura, benchmarks e dados tecnicos defensaveis
5. amadurecer recursos de engenharia proximos de software comercial

## O que evitar

- pular diretamente para "produto final"
- fingir equivalencia a software comercial proprietario
- esconder aproximacoes fisicas
- quebrar compatibilidade sem necessidade
- introduzir complexidade 3D total sem ganho fisico claro
- misturar documentacao aspiracional com funcionalidade realmente implementada
