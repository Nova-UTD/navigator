/* Typescript API bindings for /subsystems requests. */

export type PackageInfo = {
	name: string;
	executables: string[];
};

export type SubsystemInfo = {
	name: string;
	packages: PackageInfo[];
};

export type SubsystemResponse = SubsystemInfo[];

export async function subsystemList(): Promise<SubsystemResponse> {
	const response = fetch('http://127.0.0.1:8000/subsystems').then((r) => r.json());
	return response;
}
