def main():
    PathsOpen = open("Paths.yaml", "r")
    Paths = PathsOpen.readlines()

    XPosition = []
    YPosition = []

    for i in range(len(Paths)):
        if Paths[i] == "position:\n":
            X = Paths[i+1]
            Y = Paths[i+2]

            X = float(X[3:].strip())
            Y = float(Y[3:].strip())


            XPosition.append(X)
            YPosition.append(Y)

    

    print(XPosition)
    print(YPosition)
            
main()