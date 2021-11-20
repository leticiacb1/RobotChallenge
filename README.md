# Projeto de Robótica 2021.2 Reverie


## Vídeos dos objetivos cumpridos:
- Completando a pista e parando onde começou: https://www.youtube.com/watch?v=LngM1R-KVNE&ab_channel=LorranLopes
- Completa goal1 = ("blue", 12, "dog"): https://www.youtube.com/watch?v=BtXm2OVxoFc&ab_channel=LorranLopes
- Completa goal2 = ("green", 23, "car"): https://www.youtube.com/watch?v=-6uwfAVnanQ&ab_channel=LorranLopes
- Completa goal3 = ("orange", 11, "cow"): https://www.youtube.com/watch?v=l2VzrNyJFoY&ab_channel=LorranLopes
 
## Integrantes:

- Letícia Côelho
- Lorran Caetano  
- Matheus Oliveira
- Ykaro Andrade

## Status do Projeto: Concluído:heavy_check_mark:

## Features realizadas: 
- Programação orientada a objetos,
- Controle Derivativo  
- Node prestando serviço (estacao.py)

## Conceitos atingidos:
* Conceito I : Atingido :heavy_check_mark:
* Conceito C : Atingido :heavy_check_mark:
* Conceito C+ : Atingido :heavy_check_mark:
* Conceito B : Atingido :heavy_check_mark:
* Conceito B+ : Atingido :heavy_check_mark:
* Conceito A : Atingido :heavy_check_mark:

*Para ter acesso completo as funcionalidades deste repositório, caso não possua algumas das bibliotecas, realizar:*
> pip install pyfiglet
> pip install sklearn

## :warning: Para execução correta do programa siga o tutorial abaixo: :warning:

*Para poder ver a simulação basta realizar o clone do repositório em um terminal, entrar na pasta scripts e executar o simulador:*
```bash
cd catkin_ws/src 

git clone https://github.com/insper-classroom/projeto-de-robotica-reverie

cd projeto_reverie/scripts

roslaunch my_simulation trevo.launch
```

*Em outro terminal, execute o seguinte comando para permitir a execução dos comandos da garra:*
```bash

roslaunch mybot_description mybot_control2.launch 

```

*Abra outro terminal para executar o node de prestação de serviço de identificação de estação via Rede neural, rode comando abaixo, confira se está na pasta scripts:*
```bash

rosrun projeto_reverie estacao.py

```

*Finalmente, rode o comando abaixo e escolha a situação a ser simulada dentro do programa,confira se está na pasta scripts:*
```bash

rosrun projeto_reverie robo.py

```

# Objetivos 

Cores válidas do creeper: blue, green, orange
Estações válidas: dog, horse, cow e car

@2021, Insper, Terceiro Semestre, Engenharia da Computação.

