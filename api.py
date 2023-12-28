import requests

HEADERS = {'Accept': 'application/json;charset=UTF-8;qs=0.09', 'Content-Type': 'application/json'}

class Client:
    def __init__(self, base_url: str, access_key: str, secret_key: str):
        self._session = requests.Session()
        self._session.auth = (access_key, secret_key)
        self._base_url = base_url

    def _api_request(self, method: str, path: str, params={}, body={}) -> requests.Response:
        return self._session.request(method, self._base_url + path, params=params, headers=HEADERS, data=body, allow_redirects=False)
    
    def get_assembly(self, did: str, wid: str, eid: str) -> dict:
        return self._api_request('get', f'/assemblies/d/{did}/w/{wid}/e/{eid}', params={'includeMateFeatures': 'true', 'includeNonSolids': 'true', 'includeMateConnectors': 'true'}).json()
    
    def get_assembly_features(self, did: str, wid: str, eid: str) -> dict:
        return self._api_request('get', f'/assemblies/d/{did}/w/{wid}/e/{eid}/features').json()
    
    def get_parts_stl(self, did: str, mid: str, eid: str, pid: str) -> bytes:
        redirect = self._api_request('get', f'/parts/d/{did}/m/{mid}/e/{eid}/partid/{pid}/stl', params={'mode': 'binary', 'units': 'meter'}).headers['Location']
        return self._session.get(redirect).content
    
    def get_mass_properties(self, did: str, mid: str, eid: str, pid: str) -> dict:
        return self._api_request('get', f'/parts/d/{did}/m/{mid}/e/{eid}/partid/{pid}/massproperties', params={'useMassPropertyOverrides': 'true'}).json()
