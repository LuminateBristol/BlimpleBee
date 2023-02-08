from bottle import run, request, post, route

def fuckoff():
    return 'fuckoff'

def runserver():
    @post('/')
    def index():
        data = request.body.read()
        print(data)
        response = fuckoff()
        return(response)

    run(host='0.0.0.0', port=8090, debug=True)

runserver()