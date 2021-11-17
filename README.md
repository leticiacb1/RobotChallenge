# Projeto de Róbotica

Projeto de Robótica 2021.2

*Integrantes*:

- Letícia Côelho
- Lorran Caetano  
- Matheus Oliveira
- Ykaro Andrade

> Status do Projeto: Em desenvolvimento :warning: 

> Features realizadas: Programação orientada a objetos,Controle Derivativo e Node prestando serviço (estacao.py):warning: 

* Conceito I : Atingido :heavy_check_mark:
* Conceito C : Atingido :heavy_check_mark:
* Conceito C+ : Atingido :heavy_check_mark:
* Conceito B : Atingido :heavy_check_mark:
* Conceito B+ : Atingido :heavy_check_mark:
* Conceito A : Buscando :warning:

*Para ter acesso completo as funcionalidades deste repositório, caso não possua algumas das bibliotecas, realizar:*
> pip install -r requirements.txt

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

*Abra outro terminal para executar o node de prestação de serviço de identificação de estação via Rede neural, rode comando abaixo:*
```bash

rosrun projeto_reverie estacao.py

```

*Finalmente, rode o comando abaixo e escolha a situação a ser simulada dentro do programa:*
```bash

rosrun projeto_reverie robo.py

```

# Objetivos 

Cores válidas do creeper: blue, green, orange
Estações válidas: dog, horse, cow e car


# Objetivos que devem ser filmados 

```python
goal1 = ("blue", 12, "dog")

goal2 = ("green", 23, "car")

goal3 = ("orange", 11, "cow")
```

